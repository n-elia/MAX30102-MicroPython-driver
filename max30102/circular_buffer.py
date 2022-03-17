from ucollections import deque


class CircularBuffer(object):
    ''' Very simple implementation of a circular buffer based on deque '''
    def __init__(self, max_size):
        self.data = deque((), max_size, True)
        self.max_size = max_size

    def __len__(self):
        return len(self.data)

    def is_empty(self):
        return not bool(self.data)

    def append(self, item):
        try:
            self.data.append(item)
        except IndexError:
            # deque full, popping 1st item out
            self.data.popleft()
            self.data.append(item)

    def pop(self):
        return self.data.popleft()

    def clear(self):
        self.data = deque((), self.max_size, True)

    def pop_head(self):
        buffer_size = len(self.data)
        temp = self.data
        if buffer_size == 1:
            pass
        elif buffer_size > 1:
            self.data.clear()
            for x in range(buffer_size - 1):
                self.data = temp.popleft()
        else:
            return 0
        return temp.popleft()