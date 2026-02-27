/*!
 * @file
 * @brief
 */

#include <stddef.h>
#include "hsm.h"

#define top NULL

static hsm_state_t parent_of(hsm_t* self, hsm_state_t child)
{
  for(uint8_t i = 0; i < self->configuration->state_count; i++) {
    if(self->configuration->states[i].state == child) {
      return self->configuration->states[i].parent;
    }
  }

  return top;
}

static uint8_t distance_between(hsm_t* self, hsm_state_t child, hsm_state_t parent)
{
  uint8_t distance = 0;
  hsm_state_t current = child;

  while(current != parent) {
    distance++;
    current = parent_of(self, current);
  }

  return distance;
}

static hsm_state_t nth_parent(hsm_t* self, hsm_state_t state, uint8_t n)
{
  hsm_state_t current = state;

  for(uint8_t i = 0; i < n; i++) {
    current = parent_of(self, current);
  }

  return current;
}

static void send_entries(hsm_t* self, hsm_state_t after, hsm_state_t to)
{
  if(after == to) {
    return;
  }

  for(uint8_t n = (uint8_t)(distance_between(self, to, after) - 1); n > 0; n--) {
    nth_parent(self, to, n)(self, HSM_SIG_ENTRY, NULL);
  }

  to(self, HSM_SIG_ENTRY, NULL);
}

static void send_exits(hsm_t* self, hsm_state_t from, hsm_state_t before)
{
  hsm_state_t current = from;

  while(current != before) {
    current(self, HSM_SIG_EXIT, NULL);
    current = parent_of(self, current);
  }
}

static hsm_state_t nearest_common_ancestor_of(hsm_t* self, hsm_state_t a, hsm_state_t b)
{
  while(a != top) {
    hsm_state_t bb = b;

    while(bb != top) {
      if(a == bb) {
        return a;
      }

      bb = parent_of(self, bb);
    }

    a = parent_of(self, a);
  }

  return top;
}

void HSM_Init(
  hsm_t* self,
  const hsm_configuration_t* configuration,
  hsm_state_t initial)
{
  self->configuration = configuration;
  self->current = initial;

  send_entries(self, NULL, initial);
}

void HSM_SendSignal(hsm_t* self, hsm_sig_t signal, const void* data)
{
  hsm_state_t current = self->current;

  while(current != top) {
    if(current(self, signal, data) == HSM_SIG_CONSUMED) {
      return;
    }
    current = parent_of(self, current);
  }
}

void HSM_Transition(hsm_t* self, hsm_state_t target)
{
  if(self->current == target) {
    self->current(self, HSM_SIG_EXIT, NULL);
    self->current(self, HSM_SIG_ENTRY, NULL);
  }
  else {
    hsm_state_t nearest_common_ancestor = nearest_common_ancestor_of(self, self->current, target);
    send_exits(self, self->current, nearest_common_ancestor);
    self->current = target;
    send_entries(self, nearest_common_ancestor, target);
  }
}