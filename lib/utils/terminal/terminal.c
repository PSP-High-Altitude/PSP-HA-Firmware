#include "terminal.h"

#include "Regex.h"

typedef struct {
    Regex regex;
    void (*callback)(char *);

} TerminalCmd;

TerminalCmd cmd_list[TERMINAL_MAX_CMDS];
int num_cmds = 0;

void terminal_init() {}

void terminal_add_cmd(const char *cmd_regex, void (*callback)(char *)) {
    // Too many commands
    if (num_cmds >= TERMINAL_MAX_CMDS) {
        return;
    }

    // Null callback
    if (callback == NULL) {
        return;
    }

    regexCompile(&cmd_list[num_cmds].regex, cmd_regex);
    cmd_list[num_cmds].callback = callback;
    num_cmds++;
}

void terminal_process(char *str) {
    // Null buffer
    if (str == NULL) {
        return;
    }

    // Null commands
    if (num_cmds <= 0) {
        return;
    }

    // Find the first matching command
    Matcher match;
    for (int i = 0; i < num_cmds; i++) {
        match = regexMatch(&cmd_list[i].regex, str);
        if (match.isFound) {
            cmd_list[i].callback(str);
            return;
        }
    }
}