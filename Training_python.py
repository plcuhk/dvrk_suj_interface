# *** No1 ***
class Solution:
    def findNumberWithSum(self, array, test_sum):
        if (array is None) or (len(array) <= 0) or (array[-1] + array[-2] < test_sum):
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
        print('No Find of such numbers!')


testArray = [1, 23, 43, 54, 54]
Sum = 24
Test = Solution()
print(Test.findNumberWithSum(testArray, Sum))
print('*** No1 END ***')

# *** No2 ***
a_num_no2 = 1
a_str_no2 = []


def fun_no2(a_num, a_str):
    a_num = 100
    a_str.append(100)


fun_no2(a_num_no2, a_str_no2)
print(a_num_no2, a_str_no2)
print('*** No2 END ***')

# *** No3 ***
def foo_no3(x):
    print('executing foo_no3(%s)' % x)


class A:
    def foo(self, x):
        print('executing foo(%s)' % x)

    @classmethod
    def class_foo(cls, x):
        print('executing class_foo(%s)' % x)

    @staticmethod
    def static_foo(x):
        print('executing static_foo(%s)' % x)


a_no3 = A()
a_no3.foo('Hi')
a_no3.class_foo('Hi')
A.class_foo('Hi')
a_no3.static_foo('Hi')
A.static_foo('Hi')
print('*** No3 END ***')


# No4 ***
# 实例化之后，每个实例单独拥有的变量
class Test4:
    num_of_instance = 0

    def __init__(self, name):
        self.name = name
        Test4.num_of_instance += 1


print(Test4.num_of_instance)
test_4_1 = Test4('Name1')
print(Test4.num_of_instance)
test_4_2 = Test4('Name2')
print(Test4.num_of_instance)
test_4_2.num_of_instance = 100

print(test_4_1.num_of_instance)
print(test_4_2.num_of_instance)
print(Test4.num_of_instance)
