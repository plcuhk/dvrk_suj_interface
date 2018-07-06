# 增加的一个类属性用于统计Student的数量，每创建一个实例，该属性自动加一

class Student(object):
    count = 0

    def __init__(self, name, score):
        Student.name = name
        Student.score = score
        if Student.name != []:
            Student.count += 1
# 测试:
if Student.count != 0:
    print('测试失败!')
else:
    bart = Student('Bart', 90)
    if Student.count != 1:
        print('测试失败!')
    else:
        lisa = Student('Bart',80)
        if Student.count != 2:
            print('测试失败!')
        else:
            print('Students:', Student.count)
            print('测试通过!')

Michael = Student('Michael', 90)
Jane = Student('Jane', 90)
KangKang = Student('KangKang', 90)  # 说明每当创建一个实例并不会执行初始化语句 count = 0，仅在首次执行，但__init__()函数
                                    # 每创建一个实例都会执行
print(Student.count)

