def trim(s):
	start = 0
	end = -1
	slen = len(s)
	if slen == 0:
		return ''
	while s[start] == ' ':
			start += 1
			if start == slen:
				return ''
	s = s[start:]	
	while s[end] == ' ':
		end -= 1
	if end != -1:
		s = s[:end+1]
	return s
# 测试: 
if trim('hello  ') != 'hello':
    print('测试失败!')
elif trim('  hello') != 'hello':
	#print(trim('   hello'))
    print('测试失败!')
elif trim('  hello  ') != 'hello':
	#print(trim('  hello  '))
    print('测试失败!')
elif trim('  hello  world  ') != 'hello  world':
	#print(trim('   hello   world  '))
    print('测试失败!')
elif trim('') != '':
    print('测试失败!')
elif trim('    ') != '':
    print('测试失败!')
else:
    print('测试成功!')