#ifndef TERMINAL_H
#define TERMINAL_H

#define TERMINAL_MAX_CMDS 32

void terminal_init();

void terminal_add_cmd(const char *cmd_regex, void (*callback)(char *));

void terminal_process(char *str);

#endif  // TERMINAL_H