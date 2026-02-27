/*!
 * @file
 * @brief Hierarchical version of a finite state machine.
 *
 * Differs from an FSM in that a state can have a parent state that can be used to share
 * behavior via a mechanism similar to inheritance. The parent-child relationship between
 * states impacts both signal handling and transitions.
 *
 * In order to explore how signal handling and transitions work in an HSM, consider the
 * below state machine:
 *
 * +---------------------+ +---+
 * | A                   | | F |
 * | +-------+ +-------+ | +---+
 * | | B     | | D     | |
 * | | +---+ | | +---+ | |
 * | | | C | | | | E | | |
 * | | +---+ | | +---+ | |
 * | +-------+ +-------+ |
 * +---------------------+
 *
 * States B and D are children of A. States C and E are children of B and D, respectively.
 * State F has no children and no parents.
 *
 * Signals are always sent first to the active state. The active state can choose whether
 * to consume the signal or to defer to its parent. If the state chooses to consume the
 * signal then signal handling ends with the state. If, however, the state chooses to
 * defer then the signal will then be sent to the state's parent. At this point the parent
 * must make the same decision. Signal handling ends when the state or one of its
 * ancestors consumes the signal or the list of ancestors is exhausted.
 *
 * Assume that the state C shown above is active and a signal is sent to the state
 * machine. State C will be the first state to receive this signal. If it chooses to
 * defer then the signal will be sent to state B, its direct parent. If state B also
 * chooses to defer then the signal will finally be sent to state A. If A chooses to defer
 * then signal handling will end because it has no parent state.
 *
 * When transitioning, exit signals are sent up the ancestor chain until reaching the
 * nearest common ancestor of the current and target states. Then, entry signals are sent
 * down the ancestor chain to the target state. The nearest common ancestor does not
 * receive an exit signal nor does it receive an entry signal. There is a special case
 * when the current and target states match (a self-transition). In this scenario the
 * current state will be sent an exit and then an entry signal.
 *
 * For example, if C is the current state and E is the target state, then the nearest
 * common ancestor is state A. This means that exit signals are sent to C and B and then
 * entry signals are sent to D and E.
 *
 * If B is the current state and F is the target state, then there is no nearest common
 * ancestor, so exit signals are sent to B and A and then an entry signal is sent to F.
 *
 * If C is the current state and the target state, this exercises the special case of a
 * self-transition so C will be sent an exit signal then an entry signal.
 *
 * @note Transitions can only be initiated when a transition is not active. This means
 * that transitions can never be triggered from an exit signal handler and they can only
 * be triggered from an entry signal handler if the current state is the target state
 * (thus there are not more entry signals that are going to be sent).
 */

#ifndef HSM_H
#define HSM_H

#include <stddef.h>
#include <stdint.h>

#define HSM_NO_PARENT NULL

enum {
    HSM_SIG_ENTRY,
    HSM_SIG_EXIT,
    HSM_SIG_USER
};
typedef uint8_t hsm_sig_t;

enum {
    HSM_SIG_DEFFERRED,
    HSM_SIG_CONSUMED,
};
typedef uint8_t hsm_result_t;

struct hsm_t;

typedef hsm_result_t (*hsm_state_t)(
    struct hsm_t *hsm, 
    hsm_sig_t sig, 
    const void *data
);

/*!
 * Configures the parent of each state. Use NULL for the parent to indicate that a state
 * has no parent.
 */
typedef struct {
    hsm_state_t state;
    hsm_state_t parent;
} hsm_state_descriptor_t;

typedef struct {
  const hsm_state_descriptor_t* states;
  uint8_t state_count;
} hsm_configuration_t;

typedef struct hsm_t {
  const hsm_configuration_t* configuration;
  hsm_state_t current;
} hsm_t;

/*!
 * Initializes an HSM with the specified initial state. Sends entry signals down the
 * ancestor chain to the initial state.
 */
void HSM_Init(
  hsm_t* self,
  const hsm_configuration_t* configuration,
  hsm_state_t initial);

/*!
 * Sends a signal and optional signal data to the current state and, potentially, to
 * all of the state's parents.
 */
void HSM_SendSignal(hsm_t* self, hsm_sig_t signal, const void* data);

/*!
 * Transitions the HSM to the target state.
 */
void HSM_Transition(hsm_t* self, hsm_state_t target);


#endif // HSM_H