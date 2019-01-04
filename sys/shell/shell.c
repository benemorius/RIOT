/*
 * Copyright (C) 2009, Freie Universitaet Berlin (FUB).
 * Copyright (C) 2013, INRIA.
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell
 * @{
 *
 * @file
 * @brief       Implementation of a very simple command interpreter.
 *              For each command (i.e. "echo"), a handler can be specified.
 *              If the first word of a user-entered command line matches the
 *              name of a handler, the handler will be called with the whole
 *              command line as parameter.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      René Kijewski <rene.kijewski@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "log.h"
#include "shell.h"
#include "shell_commands.h"

#if !defined(SHELL_NO_ECHO) || !defined(SHELL_NO_PROMPT)
#ifdef MODULE_NEWLIB
/* use local copy of putchar, as it seems to be inlined,
 * enlarging code by 50% */
static void _putchar(int c) {
    putchar(c);
}
#else
#define _putchar putchar
#endif
#endif

#define LINE_HISTORY_COUNT (2)
#define LINE_HISTORY_LENGTH (SHELL_DEFAULT_BUFSIZE)
static char line_history[LINE_HISTORY_COUNT][LINE_HISTORY_LENGTH];
static unsigned line_history_index;

static inline void print_prompt(void);
static void replace_line(const char *line_buf);
static void refresh_line(const char *line_buf);

static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    entry++;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\n", "Command", "Description");
    puts("---------------------------------------");

    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\n", entry->name, entry->desc);
                entry++;
            }
        }
    }
}

static void handle_input_line(const shell_command_t *command_list, char *line)
{
    static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    unsigned argc = 0;
    char *pos = line;
    int contains_esc_seq = 0;
    while (1) {
        if ((unsigned char) *pos > ' ') {
            /* found an argument */
            if (*pos == '"' || *pos == '\'') {
                /* it's a quoted argument */
                const char quote_char = *pos;
                do {
                    ++pos;
                    if (!*pos) {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                    else if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                        continue;
                    }
                } while (*pos != quote_char);
                if ((unsigned char) pos[1] > ' ') {
                    puts(INCORRECT_QUOTING);
                    return;
                }
            }
            else {
                /* it's an unquoted argument */
                do {
                    if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                    }
                    ++pos;
                    if (*pos == '"') {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                } while ((unsigned char) *pos > ' ');
            }

            /* count the number of arguments we got */
            ++argc;
        }

        /* zero out the current position (space or quotation mark) and advance */
        if (*pos > 0) {
            *pos = 0;
            ++pos;
        }
        else {
            break;
        }
    }
    if (!argc) {
        return;
    }

    /* then we fill the argv array */
    char *argv[argc + 1];
    argv[argc] = NULL;
    pos = line;
    for (unsigned i = 0; i < argc; ++i) {
        while (!*pos) {
            ++pos;
        }
        if (*pos == '"' || *pos == '\'') {
            ++pos;
        }
        argv[i] = pos;
        while (*pos) {
            ++pos;
        }
    }
    for (char **arg = argv; contains_esc_seq && *arg; ++arg) {
        for (char *c = *arg; *c; ++c) {
            if (*c != '\\') {
                continue;
            }
            for (char *d = c; *d; ++d) {
                *d = d[1];
            }
            if (--contains_esc_seq == 0) {
                break;
            }
        }
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}

static int readline(char *buf, size_t size)
{
    char *line_buf_ptr = buf;
    *line_buf_ptr = '\0';

    while (1) {
        int c = getchar();
        if (c < 0) {
            return EOF;
        }

        /* We allow Unix linebreaks (\n), DOS linebreaks (\r\n), and Mac linebreaks (\r). */
        /* QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio"). */
        /* DOS newlines are handled like hitting enter twice, but empty lines are ignored. */
        if (c == '\r' || c == '\n') {
#ifndef SHELL_NO_ECHO
            _putchar('\r');
            _putchar('\n');
#endif

            /* return 1 if line is empty, 0 otherwise */
            return buf[0] == '\0';
        }
        /* ^C */
        else if (c == 'C' - '@') {
            _putchar('\r');
            _putchar('\n');
#ifdef MODULE_NEWLIB
            fflush(stdout);
#endif
            return 1; // 1 for empty line
        }
        /* QEMU uses 0x7f (DEL) as backspace, while 0x08 (BS) is for most terminals */
        else if (c == 0x08 || c == 0x7f) {
            if (line_buf_ptr == buf) {
                /* The line is empty. */
                continue;
            }

            /* overwrite previous character and keep shifting left until end */
            --line_buf_ptr;
            char *line_buf_ptr_restore = line_buf_ptr;
            while (line_buf_ptr[1] != '\0') {
                *line_buf_ptr = line_buf_ptr[1];
                ++line_buf_ptr;
            }
            *line_buf_ptr = '\0';

            /* if deleted a character somewhere in the middle of the line */
            if (line_buf_ptr != line_buf_ptr_restore) {
                fputs("\033[D", stdout); /* move cursor left one */
                refresh_line(buf);
                line_buf_ptr = line_buf_ptr_restore;
                continue;
            }

            /* white-tape the character */
#ifndef SHELL_NO_ECHO
            _putchar('\b');
            _putchar(' ');
            _putchar('\b');
#endif
        }
        /* escape sequence */
        else if (c == '[' - '@') {
            if (getchar() != '[') {
                continue;
            }
            c = getchar();
            /* up or down */
            if (c == 'A' || c == 'B') {
                if (c == 'A' && line_history_index < LINE_HISTORY_COUNT - 1) {
                    strcpy(line_history[0], buf);
                    strcpy(buf, line_history[1]);
                    line_buf_ptr = buf + strlen(buf);
                    replace_line(buf);
                    line_history_index = 1;
                }
                if (c == 'B' && line_history_index > 0) {
                    strcpy(line_history[1], buf);
                    strcpy(buf, line_history[0]);
                    line_buf_ptr = buf + strlen(buf);
                    replace_line(buf);
                    line_history_index = 0;
                }
            }
            /* left or right */
            else if (c == 'D' || c == 'C') {
                int delta = c == 'D' ? -1 : 1;
                if (delta > 0 && *line_buf_ptr == '\0') {
                    continue;
                }
                if (delta < 0 && line_buf_ptr == buf) {
                    continue;
                }
                line_buf_ptr += delta;
                if (delta > 0) {
                    fputs("\033[C", stdout);
                }
                else {
                    fputs("\033[D", stdout);
                }
            }
            else {
                const char msg[] = "[shell] unhandled escape sequence";
                LOG_INFO("%s ^[[<0x%02x> (\\033[\\0%o)\n", msg, c, c);
            }
        }
        /* received new character to add to line buffer */
        else {
            /* can't add another character if line buffer is full */
            if ((line_buf_ptr - buf) >= ((int) size) - 1) {
                continue;
            }

            /* check if we're inserting somewhere before the end of the string,
             * and make room for the inserted character if so
             */
            if (*line_buf_ptr != '\0') {
                /* shift right one by one until string ends */
                char tmp;
                char *line_buf_ptr_restore = line_buf_ptr;
                while (*line_buf_ptr != '\0') {
                    tmp = c;
                    c = *line_buf_ptr;
                    *line_buf_ptr++ = tmp;
                }
                *line_buf_ptr++ = c;
                *line_buf_ptr = '\0';
                line_buf_ptr = line_buf_ptr_restore + 1;
                /* move cursor right one */
                fputs("\033[C", stdout);
                refresh_line(buf);
                continue;
            }
            else {
                *line_buf_ptr++ = c;
                *line_buf_ptr = '\0';
            }
#ifndef SHELL_NO_ECHO
            _putchar(c);
#endif
        }
#ifdef MODULE_NEWLIB
        fflush(stdout);
#endif
    }
}

static void replace_line(const char *line_buf)
{
   /* delete line */
    fputs("\033[2K\r", stdout);
    print_prompt();
    fputs(line_buf, stdout);
    fflush(stdout);
}

static void refresh_line(const char *line_buf)
{
   /* store cursor location and delete line */
    fputs("\033[s\033[2K\r", stdout);
    print_prompt();
    fputs(line_buf, stdout);
    /* restore cursor location */
    fputs("\033[u", stdout);
    fflush(stdout);
}

static inline void print_prompt(void)
{
#ifndef SHELL_NO_PROMPT
    _putchar('>');
    _putchar(' ');
#endif

#ifdef MODULE_NEWLIB
    fflush(stdout);
#endif
}

void shell_run(const shell_command_t *shell_commands, char *line_buf, int len)
{
    /* begin prompt on a new line */
    _putchar('\r');
    _putchar('\n');
    print_prompt();

    while (1) {
        int res = readline(line_buf, len);

        if (res == EOF) {
            break;
        }

        if (!res) {
            /* add command to history if it doesn't match the last command */
            // if (strcmp(line_buf, line_history[line_history_index - 1])) {
            if (1) {
                strcpy(line_history[1], line_buf);
                line_history_index = 0;
            }
            handle_input_line(shell_commands, line_buf);
        }

        print_prompt();
    }
}
