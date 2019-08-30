# The scripts in A Bite of Python
import numpy as np
import copy

# Simple example of list
shopList = ['apple', 'mango', 'carrot', 'banana']

print(len(shopList), 'item(s) on the shopping list currently.')
print('Shopping List:')
for item in shopList:
    print(item)

shopList.append('rice')
print('\nUpdated shopping list:')
for item in shopList:
    print(item)

item_1 = shopList[0]
del shopList[0]
print(item_1, 'is deleted form the shopping list. The new shopping list:')
for item in shopList:
    print(item)

# Simple example for tuple
zoo = ('wolf', 'elephant', 'penguin')
print('\nNumber of animals in the zoo is ', len(zoo))

new_zoo = ('monkey', 'dolphin', zoo)
print('Number of animals in the zoo is ', len(new_zoo))
print('All the animals in the zoo are', new_zoo)
print('Animals brought from the old zoo are', new_zoo[2])
print('Last animal brought from the old zoo is', new_zoo[2][2])

# Tuple is usually used in printing task.
age = 18
name = 'Jack'
print('\n%s is %d years old\n' % (name, age))

# Dictionary
mailBook = {'name1': 'name1@link.com',
            'name2': 'name2@link.com',
            'name3': 'name3@link.com'}

print('Currently the item(s) in the email book is', len(mailBook))
for name, email in mailBook.items():
    print('The email of %s is %s' % (name, email))

print('\nAdding new item......')
mailBook['name4'] = 'name4@link.com'
print('Currently the item(s) in the email book is', len(mailBook))
for name, email in mailBook.items():
    print('The email of %s is %s' % (name, email))

print('\nDeleting item......')
del mailBook['name1']
print('Currently the item(s) in the email book is', len(mailBook))
for name, email in mailBook.items():
    print('The email of %s is %s \n' % (name, email))

# Slice of list
for item in shopList:
    print(item, end=' ')

print('Items 1 to 3 in the shopping list:', shopList[0:3])

# assignment, shallow copy, deep copy for compound objects like list
menu = ['rice', 'meat', 'chicken', 'sausage', ['cola', 'tea']]
print('\nOriginal menu:', menu)
print('Copying menu to copy_menu by assignment......')
copy_menu = menu
print('Copying menu to copy_menu_sw by doing a full slice or method copy()......')
copy_menu_sw = menu[:]      # shallow copy or use copy_menu_slice = menu.copy()
print('Copying menu to copy_menu_dp by deepcopy()......')
copy_menu_dp = copy.deepcopy(menu)
print('\nDeleting meat from original menu......')
del menu[1]

print('menu:', menu)
print('copy_menu:', copy_menu)
print('copy_menu_sw:', copy_menu_sw)
print('copy_menu_dp', copy_menu_dp)

print('\nAdding coffee to drinks of copy_menu_sw......')
copy_menu_sw[4].append('coffee')
print('menu:', menu)
print('copy_menu:', copy_menu)
print('copy_menu_sw:', copy_menu_sw)
print('copy_menu_dp', copy_menu_dp)

print('\nAdding milk to drinks of copy_menu_dp......')
copy_menu_dp[4].append('milk')
print('menu:', menu)
print('copy_menu:', copy_menu)
print('copy_menu_sw:', copy_menu_sw)
print('copy_menu_dp', copy_menu_dp)

# Summary of the copy of compound objects (like list, tuple not simple integers or floats): little tricky.
# Assignment copy just creating a pointer to the original object
# Shallow copy by full slice or copy() method: Creating new object but inserting reference into it to the
# objects found in the original objects. That is to say that operation like deleting or appending elements
# is independent between the copy and original but operations on the elements existing in both objects will
# be reflected on each other.
# Deep copy using copy.deepcopy() method: Construction of new objects and recursively copy objects found in
# the original objects. Real though copy.

# class


class SchoolMember:
    def __init__(self, name, age):
        self.name = name
        self.age = age
        print('Initialized SchoolMember:', self.name)

    def tell(self):
        """Tell info details."""
        print('Name: %s Age: %d' % (self.name, self.age), end=' ')


class Teacher(SchoolMember):
    def __init__(self, name, age, salary):
        SchoolMember.__init__(self, name, age)
        self.salary = salary
        print('Initialized Teacher:', self.name)

    def tell(self):
        SchoolMember.tell(self)
        print('Salary:', self.salary)


class Student(SchoolMember):
    def __init__(self, name, age, marks):
        SchoolMember.__init__(self, name, age)
        self.marks = marks
        print('Initialized Student:', self.name)

    def tell(self):
        SchoolMember.tell(self)
        print('Marks:', self.marks)


t1 = Teacher('Gao', 32, 40000)
s1 = Student('Lee', 19, 90)
t1.tell()
s1.tell()
