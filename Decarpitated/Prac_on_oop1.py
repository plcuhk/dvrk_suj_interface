# -*- coding: utf-8 -*-
# usage of @property 
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
		
# custom class
class Std(object):
	def __init__(self, name, score):     
		self.name = name
		self.score = score
	def __str__(self):     # 定制类有固定名称
		return 'Student object \n name: %s  score: %d ' % (self.name, self.score )
	__repr__ = __str__
	
a1 = Std('Michael',90)