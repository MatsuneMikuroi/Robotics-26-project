class SizedLinkedList():
    """Sized linked list for storing proximity values.
    This class implements a classic linked list but with a maximum size. When the list reaches its maximum,
    the oldest value is removed to make room for the new one.
    This is useful to calculate the value of the latest proximity values without having to store all the history,
    which can be memory intensive.

    Attributes:
        size: int - The current size of the list.
        max_size: int - The maximum size of the list.
        head: _Node | None - The head of the linked list.
        tail: _Node | None - The tail of the linked list.

    Methods:
        append(prox: tuple[float]) -> None: Add a new proximity value to the list.
        getAverage() -> tuple[float]: Calculate the average of the proximity values in the list.

    Initialization:
        max_size: int - The maximum size of the list. Default is 100.
    """

    class _Node:
        """Node class for the linked list. Each node stores a proximity value and a reference to the next node."""
        def __init__(self, prox: tuple[float]):
            self.prox_l: float = prox[0];
            self.prox_r: float = prox[1];
            self.next: SizedLinkedList._Node | None = None;

    def __init__(self, max_size: int = 100):
        # Instance variables — each SizedLinkedList object has its own state
        self.size: int = 0;
        self.max_size: int = max_size;
        self.head: SizedLinkedList._Node | None = None;
        self.tail: SizedLinkedList._Node | None = None;

    def append(self, prox: tuple[float]) -> None:
        """Add a new proximity value to the list."""
        node = SizedLinkedList._Node(prox);

        if self.size == 0:
            self.head = node;
            self.tail = node;
            self.size += 1;
        elif self.size < self.max_size:
            self.tail.next = node;
            self.tail = node;
            self.size += 1;
        else:
            self.head = self.head.next;
            self.tail.next = node;
            self.tail = node;

    def getAverage(self) -> tuple[float]:
        """Calculate the average of the proximity values in the list."""
        if self.size == 0:
            return (0, 0);

        l_sum: float = 0;
        r_sum: float = 0;
        node = self.head;
        while node is not None:
            l_sum += node.prox_l;
            r_sum += node.prox_r;
            node = node.next;
        return (l_sum / self.size, r_sum / self.size);
