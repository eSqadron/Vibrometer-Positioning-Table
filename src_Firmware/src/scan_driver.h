#include "driver.h"
#include "scanner_return_codes.h"
#include "scan_buffer.h"

struct ScannerAxis {
    int start;
    int end;
    unsigned int delta;
    enum ChannelNumber channel;
};

enum ScannerAxes {
    Yaw,
    Pitch
};

enum ScannerStatus {
    Uninitialised,
    Ready,
    Scanning,
    WaitingForContinuation,
    Stopping,
    Finished,
    Error,
};

struct ScannerDefinition {
    struct ScannerAxis axes[2];

    enum ScannerStatus status;
    unsigned int wait_time;
};

scanner_return_codes_t get_current_point(struct ScanPoint *new_point);

scanner_return_codes_t move_to_next_point(void);

scanner_return_codes_t define_scanner(struct ScannerDefinition new_scanner);

scanner_return_codes_t start_scanner(void);

enum ScannerStatus get_status(void);

scanner_return_codes_t reset_scanner(void);

scanner_return_codes_t stop_scanner(void);
