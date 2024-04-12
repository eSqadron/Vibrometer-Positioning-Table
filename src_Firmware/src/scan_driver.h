#include "driver.h"
#include "scanner_return_codes.h"

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
    WaitingForDump,
    Finished,
    Error,
};

struct ScannerDefinition {
    struct ScannerAxis axes[2];

    enum ScannerStatus status;
    unsigned int wait_time;
};

scanner_return_codes_t define_scanner(struct ScannerDefinition new_scanner);

scanner_return_codes_t start_scanner(void);

enum ScannerStatus get_status(void);
