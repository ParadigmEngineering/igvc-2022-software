#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>

typedef enum
{
    BOOT,
    AUTONOMOUS,
    MANUAL
} State;

void get_next_state(char input, State **state);
void test_fsm();

int main()
{
    char input;
    State * state;
    State tmp;

    State boot = BOOT;

    state = &boot;

    while (1)
    {
        printf("\nCurrent state is: %d\n", *state);
        printf("Please enter character input: ");
        scanf("%c", &input);
        if (input == '\n' || input == EOF)
        {
            continue;
        }
        if (input == 'q' || input == 'Q')
        {
            break;
        }
        get_next_state(input, &state);
        tmp = *state;
        fflush(stdin);
        state = &tmp;
    }

    test_fsm();
    printf("Unit Test Passed");
}

// Unit test asserting correct state transitions
// returns true if pass, false otherwise
void test_fsm()
{
    char input;

    // Test setup
    State * boot;
    State boot_val = BOOT;
    State * autonomous;
    State autonomous_val = AUTONOMOUS;
    State * manual;
    State manual_val = MANUAL;

    // Boot -> Boot transition tests
    boot = &boot_val;
    input = 'p';
    get_next_state(input, &boot);
    assert(*boot == boot_val);

    boot = &boot_val;
    input = 'P';
    get_next_state(input, &boot);
    assert(*boot == boot_val);

    boot = &boot_val;
    input = 'w';
    get_next_state(input, &boot);
    assert(*boot == boot_val);

    boot = &boot_val;
    input = 'W';
    get_next_state(input, &boot);
    assert(*boot == boot_val);

    // Boot -> Autonomous transition tests
    boot = &boot_val;
    input = 'a';
    get_next_state(input, &boot);
    assert(*boot == autonomous_val);

    boot = &boot_val;
    input = 'A';
    get_next_state(input, &boot);
    assert(*boot == autonomous_val);

    // Boot -> Manual transition tests
    boot = &boot_val;
    input = 'm';
    get_next_state(input, &boot);
    assert(*boot == manual_val);

    boot = &boot_val;
    input = 'M';
    get_next_state(input, &boot);
    assert(*boot == manual_val);

    // Autonomous -> Boot transition tests
    autonomous = &autonomous_val;
    input = 'p';
    get_next_state(input, &autonomous);
    assert(*autonomous == boot_val);

    autonomous = &autonomous_val;
    input = 'P';
    get_next_state(input, &autonomous);
    assert(*autonomous == boot_val);

    autonomous = &autonomous_val;
    input = 'w';
    get_next_state(input, &autonomous);
    assert(*autonomous == boot_val);

    autonomous = &autonomous_val;
    input = 'W';
    get_next_state(input, &autonomous);
    assert(*autonomous == boot_val);

    // Manual -> Boot transition tests
    manual = &manual_val;
    input = 'p';
    get_next_state(input, &manual);
    assert(*manual == boot_val);

    manual = &manual_val;
    input = 'P';
    get_next_state(input, &manual);
    assert(*manual == boot_val);

    manual = &manual_val;
    input = 'w';
    get_next_state(input, &manual);
    assert(*manual == boot_val);

    manual = &manual_val;
    input = 'W';
    get_next_state(input, &manual);
    assert(*manual == boot_val);
}

// Get the next state based on the current state and input
void get_next_state(char input, State **state)
{
    State boot = BOOT;
    State autonomous = AUTONOMOUS;
    State manual = MANUAL;

    int curr_state = (int)**state;

    if (curr_state == (int)boot)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else if (input == 'A' || input == 'a')
        {
            *state = &autonomous;
        }

        else if (input == 'M' || input == 'm')
        {
            *state = &manual;
        }
    }

    else if (curr_state == (int)autonomous)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else
        {
            *state = &autonomous;
        }
    }

    else if (curr_state == (int)manual)
    {
        if (input == 'P' || input == 'p' || input == 'W' || input == 'w')
        {
            *state = &boot;
        }

        else
        {
            *state = &manual;
        }
    }

    else
    {
        printf("Error: Unknown State\n");
    }
}
