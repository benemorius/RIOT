/*
 * Copyright (C) 2009, 2020 Freie Universität Berlin
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
 * @author      Juan Carrano <j.carrano@fu-berlin.de>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "od.h"
#include "log.h"
#include "shell.h"
#include "shell_commands.h"

#define ETX '\x03'  /** ASCII "End-of-Text", or ctrl-C */
#define EOT '\x04'  /** ASCII "End-of-Transmit" */
#define BS  '\x08'  /** ASCII "Backspace" */
#define ESC '\x1b'  /** ASCII "Escape" */
#define DEL '\x7f'  /** ASCII "Delete" */

#ifdef MODULE_NEWLIB
    #define flush_if_needed() fflush(stdout)
#else
    #define flush_if_needed()
#endif /* MODULE_NEWLIB */

#ifndef SHELL_NO_ECHO
    #define ECHO_ON 1
#else
    #define ECHO_ON 0
#endif /* SHELL_NO_ECHO */

#ifndef SHELL_NO_PROMPT
    #define PROMPT_ON 1
#else
    #define PROMPT_ON 0
#endif /* SHELL_NO_PROMPT */

#ifdef MODULE_SHELL_COMMANDS
    #define _builtin_cmds _shell_command_list
#else
    #define _builtin_cmds NULL
#endif

#define SQUOTE '\''
#define DQUOTE '"'
#define ESCAPECHAR '\\'
#define SPACE ' '
#define TAB '\t'

#define PARSE_ESCAPE_MASK 0x4;

enum parse_state {
    PARSE_BLANK             = 0x0,

    PARSE_UNQUOTED          = 0x1,
    PARSE_SINGLEQUOTE       = 0x2,
    PARSE_DOUBLEQUOTE       = 0x3,

    PARSE_UNQUOTED_ESC      = 0x5,
    PARSE_SINGLEQUOTE_ESC   = 0x6,
    PARSE_DOUBLEQUOTE_ESC   = 0x7,
};

static int _last_exit_status;

static inline void print_prompt(void);
static void replace_line(const char *line_buf);
static void refresh_line(const char *line_buf);
static void load_line_history(char *line_history_buf, int size, int *index);
static void push_line_history(char *line_history_buf, int size);
static char *find_line_history(char *line_history_buf, int size, int *index);

static enum parse_state escape_toggle(enum parse_state s)
{
    return s ^ PARSE_ESCAPE_MASK;
}

static shell_command_handler_t search_commands(const shell_command_t *entry,
                                               char *command)
{
    for (; entry->name != NULL; entry++) {
        if (strcmp(entry->name, command) == 0) {
            return entry->handler;
        }
    }
    return NULL;
}

static shell_command_handler_t find_handler(
        const shell_command_t *command_list, char *command)
{
    shell_command_handler_t handler = NULL;
    if (command_list != NULL) {
        handler = search_commands(command_list, command);
    }

    if (handler == NULL && _builtin_cmds != NULL) {
        handler = search_commands(_builtin_cmds, command);
    }

    return handler;
}

static void print_commands(const shell_command_t *entry)
{
    for (; entry->name != NULL; entry++) {
        printf("%-20s %s\n", entry->name, entry->desc);
    }
}

static void print_help(const shell_command_t *command_list)
{
    puts("Command              Description"
         "\n---------------------------------------");
    if (command_list != NULL) {
        print_commands(command_list);
    }

    if (_builtin_cmds != NULL) {
        print_commands(_builtin_cmds);
    }
}

/**
 * Break input line into words, create argv and call the command handler.
 *
 * Words are broken up at spaces. A backslash escapes the character that comes
 * after (meaning if it is taken literally and if it is a space it does not break
 * the word). Spaces can also be protected by quoting with double or single
 * quotes.
 *
 * There are two unquoted states (PARSE_BLANK and PARSE_UNQUOTED) and two quoted
 * states (PARSE_SINGLEQUOTE and PARSE_DOUBLEQUOTE). In addition, every state
 * (except PARSE_BLANK) has an escaped pair state (e.g PARSE_SINGLEQUOTE and
 * PARSE_SINGLEQUOTE_ESC).
 *
 * For the following let's define some things
 *      - Function transit(character, state) to change to 'state' after
 *        'character' was read. The order of a list of transit-functions matters.
 *      - A BLANK is either SPACE or TAB
 *      - '*' means any character
 *
 *      PARSE_BLANK
 *          transit(SQUOTE, PARSE_SINGLEQUOTE)
 *          transit(DQUOTE, PARSE_DOUBLEQUOTE)
 *          transit(ESCAPECHAR, PARSE_UNQUOTED_ESC)
 *          transit(BLANK, PARSE_BLANK)
 *          transit(*, PARSE_UNQUOTED) -> store character
 *
 *      PARSE_UNQUOTED
 *          transit(SQUOTE, PARSE_SINGLEQUOTE)
 *          transit(DQUOTE, PARSE_DOUBLEQUOTE)
 *          transit(BLANK, PARSE_BLANK)
 *          transit(ESCAPECHAR, PARSE_UNQUOTED_ESC)
 *          transit(*, PARSE_UNQUOTED) -> store character
 *
 *      PARSE_UNQUOTED_ESC
 *          transit(*, PARSE_UNQUOTED) -> store character
 *
 *      PARSE_SINGLEQUOTE
 *          transit(SQUOTE, PARSE_UNQUOTED)
 *          transit(ESCAPECHAR, PARSE_SINGLEQUOTE_ESC)
 *          transit(*, PARSE_SINGLEQUOTE) -> store character
 *
 *      PARSE_SINGLEQUOTE_ESC
 *          transit(*, PARSE_SINGLEQUOTE) -> store character
 *
 *      PARSE_DOUBLEQUOTE
 *          transit(DQUOTE, PARSE_UNQUOTED)
 *          transit(ESCAPECHAR, PARSE_DOUBLEQUOTE_ESC)
 *          transit(*, PARSE_DOUBLEQUOTE) -> store character
 *
 *      PARSE_DOUBLEQUOTE_ESC
 *          transit(*, PARSE_DOUBLEQUOTE) -> store character
 *
 *
 */
static void handle_input_line(const shell_command_t *command_list, char *line)
{
    /* first we need to calculate the number of arguments */
    int argc = 0;
    char *readpos = line;
    char *writepos = readpos;

    uint8_t pstate = PARSE_BLANK;
    if (IS_USED(MODULE_SHELL_HOOKS)) {
        shell_post_readline_hook();
    }
    for (; *readpos != '\0'; readpos++) {

        char wordbreak = SPACE;
        bool is_wordbreak = false;

        switch (pstate) {

            case PARSE_BLANK:
                if (*readpos != SPACE && *readpos != TAB) {
                    argc++;
                }

                if (*readpos == SQUOTE) {
                    pstate = PARSE_SINGLEQUOTE;
                }
                else if (*readpos == DQUOTE) {
                    pstate = PARSE_DOUBLEQUOTE;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = PARSE_UNQUOTED_ESC;
                }
                else if (*readpos != SPACE && *readpos != TAB) {
                    pstate = PARSE_UNQUOTED;
                    *writepos++ = *readpos;
                }
                break;

            case PARSE_UNQUOTED:
                if (*readpos == SQUOTE) {
                    pstate = PARSE_SINGLEQUOTE;
                }
                else if (*readpos == DQUOTE) {
                    pstate = PARSE_DOUBLEQUOTE;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = escape_toggle(pstate);
                }
                else if (*readpos == SPACE || *readpos == TAB) {
                    pstate = PARSE_BLANK;
                    *writepos++ = '\0';
                }
                else {
                    *writepos++ = *readpos;
                }
                break;

            case PARSE_SINGLEQUOTE:
                wordbreak = SQUOTE;
                is_wordbreak = true;
                break;

            case PARSE_DOUBLEQUOTE:
                wordbreak = DQUOTE;
                is_wordbreak = true;
                break;

            default: /* QUOTED state */
                pstate = escape_toggle(pstate);
                *writepos++ = *readpos;
                break;
        }

        if (is_wordbreak) {
            if (*readpos == wordbreak) {
                if (wordbreak == SQUOTE || wordbreak == DQUOTE) {
                    pstate = PARSE_UNQUOTED;
                }
            }
            else if (*readpos == ESCAPECHAR) {
                pstate = escape_toggle(pstate);
            }
            else {
                *writepos++ = *readpos;
            }
        }
    }
    *writepos = '\0';

    if (pstate != PARSE_BLANK && pstate != PARSE_UNQUOTED) {
        puts("shell: incorrect quoting");
        return;
    }

    if (argc == 0) {
        return;
    }

    /* then we fill the argv array */
    int collected;
    char *argv[argc];

    readpos = line;
    for (collected = 0; collected < argc; collected++) {
        argv[collected] = readpos;
        readpos += strlen(readpos) + 1;
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        if (IS_USED(MODULE_SHELL_HOOKS)) {
            shell_pre_command_hook(argc, argv);
            int res = handler(argc, argv);
            _last_exit_status = res;
            shell_post_command_hook(res, argc, argv);
        }
        else {
            _last_exit_status = handler(argc, argv);
        }
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
            _last_exit_status = 0;
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
            _last_exit_status = 1;
        }
    }
}

__attribute__((weak)) void shell_post_readline_hook(void)
{

}

__attribute__((weak)) void shell_pre_command_hook(int argc, char **argv)
{
    (void)argv;
    (void)argc;
}

__attribute__((weak)) void shell_post_command_hook(int ret, int argc,
                                                   char **argv)
{
    (void)ret;
    (void)argv;
    (void)argc;
}

static inline void print_prompt(void)
{
    if (PROMPT_ON) {
        if (_last_exit_status) {
            /* error prompt */
            printf("%i \033[93m>\033[0m ", _last_exit_status);
        } else {
            /* normal prompt */
            fputs("\033[91m>\033[0m ", stdout);
        }
    }

    flush_if_needed();
}

static inline void echo_char(char c)
{
    if (ECHO_ON) {
        putchar(c);
    }
}

static inline void white_tape(void)
{
    if (ECHO_ON) {
        putchar('\b');
        putchar(' ');
        putchar('\b');
    }
}

static inline void new_line(void)
{
    if (ECHO_ON) {
        putchar('\r');
        putchar('\n');
    }
}

/**
 * @brief   Read a single line from standard input into a buffer.
 *
 * In addition to copying characters, this routine echoes the line back to
 * stdout and also supports primitive line editing.
 *
 * If the input line is too long, the input will still be consumed until the end
 * to prevent the next line from containing garbage.
 *
 * We allow Unix (\n), DOS (\r\n), and Mac linebreaks (\r).
 * QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio").
 * DOS newlines are handled like hitting enter twice.
 *
 * @param   buf     Buffer where the input will be placed.
 * @param   size    Size of the buffer. The maximum line length will be one less
 *                  than size, to accommodate for the null terminator.
 *                  The minimum buffer size is 1.
 *
 * @return  length of the read line, excluding the terminator, if reading was
 *          successful.
 * @return  EOF, if the end of the input stream was reached.
 * @return  -ENOBUFS if the buffer size was exceeded.
 */
static int readline(char *buf, size_t size)
{
    assert((size_t) size > 0);
    char *line_buf_ptr = buf;
    *line_buf_ptr = '\0';
    int line_history_index = -1;

    while (1) {
        int c = getchar();

        switch (c) {

            case EOT:
                /* ignore if line isn't blank */
                if (line_buf_ptr != buf) {
                    continue;
                }
                /* fall-thru */
            case EOF:
                return EOF;

            /* Ctrl-C cancels the current line. */
            case ETX:
                new_line();
                return ETX;

            case '\r':
                /* fall-thru */
            case '\n':
                new_line();

                return line_buf_ptr - buf;

            /* check for backspace: */
            case BS:    /* 0x08 (BS) for most terminals */
                /* fall-thru */
            case DEL:   /* 0x7f (DEL) when using QEMU */
                if (line_buf_ptr == buf) {
                    /* the line is empty */
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

                /* we if deleted a character somewhere other than the
                 * end of the line then we need to refresh the line
                 */
                if (line_buf_ptr != line_buf_ptr_restore) {
                    fputs("\033[D", stdout); /* move cursor left one */
                    refresh_line(buf);
                    line_buf_ptr = line_buf_ptr_restore;
                    continue;
                }
                /* otherwise the character was deleted from the end */

                white_tape();
                break;

            /* escape sequence */
            case ESC: { /* first character is ^[ */
                c = getchar();
                if (c != '[' && c != 'O') { /* second character is [ or O */
                    continue;
                }
                c = getchar(); /* third/last character is of interest */

                /* up or down */
                if (c == 'A' || c == 'B') {
                    /* load commands from history into the current line_buf */

                    int delta = c == 'A' ? 1 : -1;
                    /* can't have line_history_index < -1 */
                    if (line_history_index + delta < -1) {
                        line_history_index = -1;
                        continue;
                    }

                    /* If line_buf contains a command and it didn't come from
                     * history, then we need to store it before overwriting
                     * line_buf with the requested history. This also means
                     * it will become a part of the command history even though
                     * the user didn't run the command. This is perhaps not
                     * desirable.
                     */
                    if (line_history_index == -1
                        && delta > 0 /* should always be true here actually */
                        && line_buf_ptr != buf
                    ) {
                        push_line_history(buf, size);
                        ++line_history_index;
                    }

                    line_history_index += delta;

                    /* going back down past the most recent history
                     * always comes back to a blank line
                     */
                    if (line_history_index == -1) {
                        line_buf_ptr = buf;
                        *line_buf_ptr = '\0';
                        replace_line(buf);
                        continue;
                    }

                    /* load the requested history into the current line_buf */
                    load_line_history(buf, size, &line_history_index);

                    /* stop at the oldest history entry */
                    if (strlen(buf) == 0) {
                        --line_history_index;
                        load_line_history(buf, size, &line_history_index);
                    }

                    replace_line(buf);
                    line_buf_ptr = buf + strlen(buf);
                }
                /* left or right */
                else if (c == 'D' || c == 'C') {
                    /* check that it's ok to move the cursor in that direction */
                    int delta = c == 'D' ? -1 : 1;
                    if (delta > 0 && *line_buf_ptr == '\0') {
                        continue;
                    }
                    if (delta < 0 && line_buf_ptr == buf) {
                        continue;
                    }

                    /* now move the cursor */
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
                break;
            }
            /* received new character to add to line buffer */
            default:
                /* unhandled control character */
                if (c < 0x20) {
                    const char msg[] = "[shell] unhandled control character";
                    LOG_DEBUG("%s ^%c (0x%02x)\n", msg, c + '@', c);
                    continue;
                }

                /* can't add another character if line buffer is full */
                if ((line_buf_ptr - buf) >= ((int) size) - 1) {
                    continue;
                }

                /* check if we're inserting somewhere before the end of the string,
                 * and make room for the inserted character if so
                 */
                if (*line_buf_ptr != '\0') {
                    /* shift right one by one until string ends, inserting
                     * the new character in the first opening
                     */
                    char tmp;
                    char *line_buf_ptr_restore = line_buf_ptr;
                    while (*line_buf_ptr != '\0') {
                        tmp = c;
                        c = *line_buf_ptr;
                        *line_buf_ptr++ = tmp;
                    }
                    *line_buf_ptr++ = c; /* while loop misses the last one */
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

                echo_char(c);
                break;
        }

        flush_if_needed();
    }
}

static void replace_line(const char *line_buf)
{
    fputs("\033[2K\r", stdout); /* delete line */
    print_prompt();
    fputs(line_buf, stdout);
    fflush(stdout);
}

static void refresh_line(const char *line_buf)
{
    fputs("\033[s\033[2K\r", stdout); /* store cursor location and delete line */
    print_prompt();
    fputs(line_buf, stdout);
    fputs("\033[u", stdout); /* restore cursor location */
    fflush(stdout);
}

static char *_find_history_front(char *line_history_buf)
{
    /* find the beginning of history (the 'e' in "echo 1") */
     char *history_ptr = strchr(line_history_buf, '\0');
     ++history_ptr;
     if (*history_ptr == '\0') {
        /* current line_buf hasn't yet collided with history
         * beginning of history will be the next nonzero character
         */
         while (*history_ptr == '\0') {
            ++history_ptr;
         }
     }
     else {
        /* line_buf has collided with history already
         * history_ptr is currently inside the overwritten command
         * beginning of undamaged history will start after the next \0
         */
         history_ptr = strchr(history_ptr, '\0');
         ++history_ptr;
     }
     /* actually this needs to point to the \0 */
     --history_ptr;
     return history_ptr;
}

static void push_line_history(char *line_history_buf, int size)
{
    /* store the active line_buf in history, but not
     * if it matches the last command already there
     */
    int history_index = 0;
    char *latest_line = find_line_history(line_history_buf, size, &history_index);
    // printf("comparing with index %i\n", history_index);
    if (latest_line[0] != '\0' && strcmp(line_history_buf, latest_line) == 0) {
        return;
    }

    /* shift bytes out the front and into the back of line_history_buf
     * until the front character is \0
     */
    char *buf_ptr;
    do {
        buf_ptr = line_history_buf;
        /* take the first character to place it at the rear */
        char front_c = *buf_ptr;

        /* shift the rest of line_history_buf left one position */
        while (buf_ptr < line_history_buf + size - 1) {
            *buf_ptr = buf_ptr[1];
            ++buf_ptr;
        }
        *buf_ptr = front_c;

    } while (*buf_ptr != '\0');
}

static char *find_line_history(char *line_history_buf, int size, int *index)
{
    /* line_history_buf currently looks like "ifconfig\0...\0echo 1\0echo 2\0" */

    /* find the *index'th command string starting from the rear
     * and copy it to the front of line_history_buf for line_buf
     */
    char *history_front = _find_history_front(line_history_buf);
    char *buf_ptr = line_history_buf + size - 1;
    // --buf_ptr; /* start one character left of the \0 */
    char *line_ptr;
    int count = 0;
    do {
        do {} while (*--buf_ptr != '\0');
        line_ptr = buf_ptr + 1;
    } while (++count <= *index && buf_ptr > history_front);
    *index = count - 1;
    return line_ptr;
}

static void load_line_history(char *line_history_buf, int size, int *index)
{
    char *line_ptr = find_line_history(line_history_buf, size, index);
    // printf("loading from index %i\n", *index);
    strcpy(line_history_buf, line_ptr);

    return;

    /* line_history_buf currently looks like "ifconfig\0\0...\0echo 1\0echo 2\0"
     * the extra \0 after ifconfig marks the true end of line_buf (including its future)
     * and the one before echo 1 marks the beginning of history
     */

}

void shell_run_once(const shell_command_t *shell_commands,
                    char *line_buf, int len)
{
    /* begin prompt on a new line */
    new_line();
    print_prompt();

    /* required by push_line_history() */
    memset(line_buf, '\0', len);

    /* handle_input_line() will turn all ' ' into '\0'
     * so give it a copy on the stack
     * and find a better solution later
     */
    char fix_me[len];

    while (1) {
        int res = readline(line_buf, len);

        switch (res) {

            case EOF:
                return;

            case ETX: /* Ctrl-C */
                break;

            case -ENOBUFS:
                puts("shell: maximum line length exceeded");
                break;

            default:
                strcpy(fix_me, line_buf);
                handle_input_line(shell_commands, fix_me);
                push_line_history(line_buf, len);
                break;
        }

        print_prompt();
    }
}
