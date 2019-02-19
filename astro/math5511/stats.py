class Stats():
    """ Helper class to track statistics (min, max, cnt, sum, avg). """
    def __init__(self):
        self.zero()

    def zero(self):
        self.__min : float = 0
        self.__max : float = 0
        self.__cnt : int = 0
        self.__sum : float = 0

    def getMin(self) -> float:
        return self.__min

    def getMax(self) -> float:
        return self.__max

    def getCount(self) -> int:
        return self.__cnt

    def getSum(self) -> float:
        return self.__sum

    def getAvg(self) -> float:
        avg: float = 0
        if self.__cnt > 0:
            avg = self.__sum / self.__cnt
        return avg

    def add(self, val: float):
        if self.__cnt == 0:
            self.__min = self.__max = val
        elif val > self.__max:
            self.__max = val
        elif val < self.__min:
            self.__min = val
        self.__cnt += 1
        self.__sum += val
