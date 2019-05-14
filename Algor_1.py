class Solution:
    def findNumberWithSum(self, array, test_sum):
        if array == None or len(array) <= 0 or array[-1] + array[-2] < test_sum:
            return []
        start = 0
        end = len(array) - 1
        while start < end:
            if array[start] + array[end] < test_sum:
                start += 1
            elif array[start] + array[end] > test_sum:
                end -= 1
            else:
                return [array[start], array[end]]


testArray = [1, 23, 43, 54, 54]
Sum = 108
Test = Solution()
print(Test.findNumberWithSum(testArray, Sum))


