# The scripts in A Bite of Python
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
