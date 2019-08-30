# Prac on sorted()

L = [('Bob', 75), ('Adam', 92), ('Bart', 66), ('Lisa', 88)]

def by_name(t):
	return t[0].lower()

def by_score(t):
	return t[1]

L1 = sorted(L, key = by_name)
print(L1)

L2 = sorted(L, key = by_score, reverse = True)
print(L2)

class Screen(object):
	@property
	def width(self):
		return Screen._width
	
	@width.setter
	def width(self, w):
		if not isinstance(w, int):
			raise ValueError('Not an interger!')
		if w <= 0:
			raise ValueError('Not right range!')
		Screen._width = w

	@property
	def test(self):
		return Screen._width + 100