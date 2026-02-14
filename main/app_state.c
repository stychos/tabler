#include <string.h>
#include "app_state.h"

static app_state_t state;

void app_state_init(void)
{
    memset(&state, 0, sizeof(state));
}

app_state_t *app_state_get(void)
{
    return &state;
}

void app_state_clear(void)
{
    memset(&state, 0, sizeof(state));
}
