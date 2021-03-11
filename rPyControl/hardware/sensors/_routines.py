# TODO
# Implement low pass filter


def counter_overflow(counts, prev_counts, buffer, threshold=None):
    """handle overflow of counters with given scale and threshold"""

    turns = 0

    if not threshold:
        threshold = buffer / 2

    if prev_counts - counts >= threshold:
        turns += 1
    elif prev_counts - counts <= -1 * threshold:
        turns -= 1

    return turns