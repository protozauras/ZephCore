/* DiagRing implementation — see DiagRing.h. */
#include <mesh/DiagRing.h>
#include <string.h>

#define DIAG_RING_SLOTS 8

struct diag_slot {
	uint32_t uptime_ms;
	uint8_t len;
	char text[DIAG_RING_LINE_MAX];
};

static struct diag_slot ring[DIAG_RING_SLOTS];
static uint8_t head;   /* next write */
static uint8_t tail;   /* next read (pending while tail != head) */
static uint8_t seq;

void diag_ring_add(uint32_t uptime_ms, const char *text)
{
	if (text == NULL) {
		return;
	}
	struct diag_slot *s = &ring[head];

	s->uptime_ms = uptime_ms;
	s->len = (uint8_t)strnlen(text, DIAG_RING_LINE_MAX);
	memcpy(s->text, text, s->len);

	head = (uint8_t)((head + 1) % DIAG_RING_SLOTS);
	if (head == tail) {
		/* Full: drop the oldest. */
		tail = (uint8_t)((tail + 1) % DIAG_RING_SLOTS);
	}
}

bool diag_ring_pending(void)
{
	return tail != head;
}

int diag_ring_build(uint8_t *out, int cap)
{
	if (!diag_ring_pending() || out == NULL || cap < 6) {
		return 0;
	}

	const struct diag_slot *s = &ring[tail];
	const int text_max = (cap - 6) < DIAG_RING_LINE_MAX
				     ? (cap - 6)
				     : DIAG_RING_LINE_MAX;
	const uint8_t text_len = s->len < (uint8_t)text_max
					 ? s->len
					 : (uint8_t)text_max;

	out[0] = 0xD1;
	out[1] = seq++;
	out[2] = (uint8_t)(s->uptime_ms);
	out[3] = (uint8_t)(s->uptime_ms >> 8);
	out[4] = (uint8_t)(s->uptime_ms >> 16);
	out[5] = (uint8_t)(s->uptime_ms >> 24);
	memcpy(&out[6], s->text, text_len);

	tail = (uint8_t)((tail + 1) % DIAG_RING_SLOTS);
	return 6 + (int)text_len;
}
