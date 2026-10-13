// rfid_dc.cpp - IgH EtherCAT master for PIC32 RFID slave, DC SYNC0 mode, 1 ms cycle
//
// Build:  g++ -O2 -Wall -o rfid_dc rfid_dc.cpp -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat
// Run:    sudo ./rfid_dc 0

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include "ecrt.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define VENDOR_ID        0x0000007b
#define PRODUCT_CODE     0x00009252

#define CYCLE_NS         1000000U          // 1 ms cycle
#define SYNC0_SHIFT_NS   (CYCLE_NS / 2)    // SYNC0 fires 500 us after frame send (tune)
#define ASSIGN_ACTIVATE  0x0300            // SYNC0 enable; check <AssignActivate> in your ESI
#define RT_PRIORITY      80
#define PRINT_EVERY      1000              // print once per second (1000 cycles)

#define NSEC_PER_SEC     1000000000ULL
#define TIMESPEC2NS(T)   ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (uint64_t)(T).tv_nsec)

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static ec_master_t        *master  = NULL;
static ec_domain_t        *domain1 = NULL;
static ec_slave_config_t  *sc      = NULL;
static uint8_t            *pd      = NULL;

static ec_master_state_t       master_state = {};
static ec_domain_state_t       domain1_state = {};
static ec_slave_config_state_t sc_state = {};

static volatile sig_atomic_t run = 1;

// ---------------------------------------------------------------------------
// Process data offsets (one per PDO entry)
// ---------------------------------------------------------------------------
// Outputs  (master -> slave), 0x7000
static unsigned int o_control, o_header, o_hsptl_code, o_num_usage;
static unsigned int o_last_date, o_last_month, o_last_year, o_surgical_id;
// Inputs   (slave -> master), 0x6000
static unsigned int i_status, i_header, i_instr_type, i_usage_limit, i_instr_version;
static unsigned int i_country, i_plant, i_mfg_date, i_mfg_month, i_mfg_year, i_qc;
static unsigned int i_hsptl_code, i_num_usage, i_last_date, i_last_month, i_last_year;
static unsigned int i_surgical_id;

// ---------------------------------------------------------------------------
// PDO mapping (must match the slave's ESI / `ethercat pdos` output)
// ---------------------------------------------------------------------------
static ec_pdo_entry_info_t slave_pdo_entries[] = {
    // RxPDO 0x1600 (outputs)
    {0x7000, 0x01,  8}, // Control_word
    {0x7000, 0x02,  8}, // Header
    {0x7000, 0x03, 16}, // Used_hsptl_code
    {0x7000, 0x04,  8}, // Number_of_usage
    {0x7000, 0x05,  8}, // Last_used_Date
    {0x7000, 0x06,  8}, // Last_used_Month
    {0x7000, 0x07,  8}, // Last_used_Year
    {0x7000, 0x08, 16}, // Active_Surgical_ID
    // TxPDO 0x1A00 (inputs)
    {0x6000, 0x01,  8}, // Status_word
    {0x6000, 0x02,  8}, // Header
    {0x6000, 0x03,  8}, // Instrument_type
    {0x6000, 0x04,  8}, // Usage_limit
    {0x6000, 0x05, 16}, // Instrument_version
    {0x6000, 0x06,  8}, // Mfg_Country_code
    {0x6000, 0x07,  8}, // Mfg_Plant_code
    {0x6000, 0x08,  8}, // Mfg_Date
    {0x6000, 0x09,  8}, // Mfg_Month
    {0x6000, 0x10,  8}, // Mfg_year
    {0x6000, 0x11,  8}, // QC_Status
    {0x6000, 0x12, 16}, // Used_hsptl_code
    {0x6000, 0x13,  8}, // Number_of_usage
    {0x6000, 0x14,  8}, // Last_used_Date
    {0x6000, 0x15,  8}, // Last_used_Month
    {0x6000, 0x16,  8}, // Last_used_Year
    {0x6000, 0x17, 16}, // Active_Surgical_ID
};

static ec_pdo_info_t slave_pdos[] = {
    {0x1600,  8, slave_pdo_entries + 0},
    {0x1a00, 17, slave_pdo_entries + 8},
};

static ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,           EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,           EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void signal_handler(int) { run = 0; }

static struct timespec timespec_add_ns(struct timespec t, uint32_t ns)
{
    t.tv_nsec += ns;
    while (t.tv_nsec >= (long)NSEC_PER_SEC) {
        t.tv_nsec -= NSEC_PER_SEC;
        t.tv_sec++;
    }
    return t;
}

static void setup_realtime(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        perror("mlockall (warning)");

    struct sched_param param = {};
    param.sched_priority = RT_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
        perror("sched_setscheduler SCHED_FIFO (warning, run as root)");

    // Pre-fault stack so no page faults happen inside the loop
    unsigned char dummy[8 * 1024];
    memset(dummy, 0, sizeof(dummy));
}

static void check_states(bool print)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter || ds.wc_state != domain1_state.wc_state)
        if (print) printf("Domain1: WC %u, state %u\n", ds.working_counter, ds.wc_state);
    domain1_state = ds;

    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("Master: %u slave(s) responding\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("Master: AL states 0x%02X\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Master: link %s\n", ms.link_up ? "up" : "down");
    master_state = ms;

    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc, &s);
    if (s.al_state != sc_state.al_state)
        printf("Slave: AL state 0x%02X\n", s.al_state);
    if (s.online != sc_state.online)
        printf("Slave: %s\n", s.online ? "online" : "offline");
    if (s.operational != sc_state.operational)
        printf("Slave: %soperational\n", s.operational ? "" : "not ");
    sc_state = s;
}

// ---------------------------------------------------------------------------
// Cyclic process-data handling (called once per 1 ms cycle)
// ---------------------------------------------------------------------------
static void process_data(uint32_t cycle)
{
    // ---- read inputs ----
    uint8_t  status       = EC_READ_U8 (pd + i_status);
    uint8_t  header       = EC_READ_U8 (pd + i_header);
    uint8_t  instr_type   = EC_READ_U8 (pd + i_instr_type);
    uint8_t  usage_limit  = EC_READ_U8 (pd + i_usage_limit);
    uint16_t instr_ver    = EC_READ_U16(pd + i_instr_version);
    uint8_t  country      = EC_READ_U8 (pd + i_country);
    uint8_t  plant        = EC_READ_U8 (pd + i_plant);
    uint8_t  mfg_d        = EC_READ_U8 (pd + i_mfg_date);
    uint8_t  mfg_m        = EC_READ_U8 (pd + i_mfg_month);
    uint8_t  mfg_y        = EC_READ_U8 (pd + i_mfg_year);
    uint8_t  qc           = EC_READ_U8 (pd + i_qc);
    uint16_t hsptl        = EC_READ_U16(pd + i_hsptl_code);
    uint8_t  n_usage      = EC_READ_U8 (pd + i_num_usage);
    uint8_t  lu_d         = EC_READ_U8 (pd + i_last_date);
    uint8_t  lu_m         = EC_READ_U8 (pd + i_last_month);
    uint8_t  lu_y         = EC_READ_U8 (pd + i_last_year);
    uint16_t surg_id      = EC_READ_U16(pd + i_surgical_id);

    // ---- write outputs ----
    EC_WRITE_U8 (pd + o_control,     75);   // 0x4B
    EC_WRITE_U8 (pd + o_header,      26);
    EC_WRITE_U16(pd + o_hsptl_code,  0);
    EC_WRITE_U8 (pd + o_num_usage,   0);
    EC_WRITE_U8 (pd + o_last_date,   0);
    EC_WRITE_U8 (pd + o_last_month,  0);
    EC_WRITE_U8 (pd + o_last_year,   0);
    EC_WRITE_U16(pd + o_surgical_id, 0);

    // Printing blocks; keep it rare inside a 1 ms loop
    if (cycle % PRINT_EVERY == 0) {
        printf("Status 0x%02X Hdr %u Type %u UsageLim %u Ver %u | Mfg %u/%u C%u P%u %02u-%02u-%02u QC %u | "
               "Hosp %u Uses %u Last %02u-%02u-%02u SurgID %u\n",
               status, header, instr_type, usage_limit, instr_ver,
               country, plant, 0, 0, mfg_d, mfg_m, mfg_y, qc,
               hsptl, n_usage, lu_d, lu_m, lu_y, surg_id);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <slave_position>\n", argv[0]);
        return EXIT_FAILURE;
    }
    char *end = NULL;
    errno = 0;
    unsigned long pos = strtoul(argv[1], &end, 10);
    if (argv[1][0] == '\0' || *end != '\0' || errno == ERANGE || pos > UINT16_MAX) {
        fprintf(stderr, "Invalid slave position: %s\n", argv[1]);
        return EXIT_FAILURE;
    }
    const uint16_t slave_pos = (uint16_t)pos;

    const ec_pdo_entry_reg_t regs[] = {
        // outputs
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x01, &o_control},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x02, &o_header},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x03, &o_hsptl_code},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x04, &o_num_usage},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x05, &o_last_date},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x06, &o_last_month},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x07, &o_last_year},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x7000, 0x08, &o_surgical_id},
        // inputs
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x01, &i_status},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x02, &i_header},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x03, &i_instr_type},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x04, &i_usage_limit},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x05, &i_instr_version},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x06, &i_country},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x07, &i_plant},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x08, &i_mfg_date},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x09, &i_mfg_month},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x10, &i_mfg_year},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x11, &i_qc},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x12, &i_hsptl_code},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x13, &i_num_usage},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x14, &i_last_date},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x15, &i_last_month},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x16, &i_last_year},
        {0, slave_pos, VENDOR_ID, PRODUCT_CODE, 0x6000, 0x17, &i_surgical_id},
        {}
    };

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "Failed to request master\n"); return EXIT_FAILURE; }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) { fprintf(stderr, "Failed to create domain\n"); goto fail; }

    sc = ecrt_master_slave_config(master, 0, slave_pos, VENDOR_ID, PRODUCT_CODE);
    if (!sc) { fprintf(stderr, "Failed to get slave config\n"); goto fail; }

    if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs)) {
        fprintf(stderr, "Failed to configure PDOs\n"); goto fail;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, regs)) {
        fprintf(stderr, "PDO entry registration failed\n"); goto fail;
    }

    // ---- Distributed Clocks: SYNC0, 1 ms ----
    // Optional: some SSC slaves want the sync mode set explicitly (0x1C32:01 = 2 -> DC SYNC0).
    // Enable only if your object dictionary has 0x1C32 and allows writing it.
    // ecrt_slave_config_sdo16(sc, 0x1C32, 0x01, 0x0002);
    ecrt_slave_config_dc(sc, ASSIGN_ACTIVATE, CYCLE_NS, SYNC0_SHIFT_NS, 0, 0);

    // Reference clock = this slave (default would be the first DC-capable slave anyway)
    if (ecrt_master_select_reference_clock(master, sc))
        fprintf(stderr, "Warning: could not select reference clock\n");

    {
        // Give the master an initial application time before activation (DC start time calc)
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        ecrt_master_application_time(master, TIMESPEC2NS(t));
    }

    if (ecrt_master_activate(master)) { fprintf(stderr, "Failed to activate master\n"); goto fail; }

    pd = ecrt_domain_data(domain1);
    if (!pd) { fprintf(stderr, "Failed to get domain data\n"); goto fail; }

    setup_realtime();
    printf("Running DC SYNC0 @ %u us, shift %u us. Ctrl+C to stop.\n",
           CYCLE_NS / 1000, SYNC0_SHIFT_NS / 1000);

    {
        struct timespec wakeup, now;
        uint32_t cycle = 0;
        unsigned int sync_ref_counter = 0;

        clock_gettime(CLOCK_MONOTONIC, &wakeup);

        while (run) {
            // 1) Sleep until the next absolute cycle start (no drift, unlike usleep)
            wakeup = timespec_add_ns(wakeup, CYCLE_NS);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

            // 2) Receive and process inputs
            ecrt_master_receive(master);
            ecrt_domain_process(domain1);

            if (cycle % PRINT_EVERY == 0)
                check_states(true);

            // 3) Application logic
            process_data(cycle);

            // 4) Queue outputs + DC synchronisation datagrams
            ecrt_domain_queue(domain1);

            clock_gettime(CLOCK_MONOTONIC, &now);
            ecrt_master_application_time(master, TIMESPEC2NS(now));

            if (sync_ref_counter) {
                sync_ref_counter--;
            } else {
                sync_ref_counter = 1;                 // sync reference clock every 2nd cycle
                ecrt_master_sync_reference_clock(master);
            }
            ecrt_master_sync_slave_clocks(master);

            // 5) Send
            ecrt_master_send(master);
            cycle++;
        }
    }

    printf("Stopping, releasing master.\n");
    ecrt_release_master(master);
    return EXIT_SUCCESS;

fail:
    ecrt_release_master(master);
    return EXIT_FAILURE;
}
