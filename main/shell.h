#pragma once

#include "rc522.h"

// Start the interactive shell loop (blocks forever)
void shell_run(rc522_handle_t *rc522);
