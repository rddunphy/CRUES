def convert(file):
	out = []
	try:
		with open(file) as f:
			for l in f:
			  	if l == "---\n":
					continue
				else:
					out += [int(l.split(": ", 2)[1][0:-1])]
	except IOError:
		pass
	return out

def mean(xs):
	sum, count = 0, 0
	for x in xs:
		if x != -1:
			sum += x
			count += 1
	if count > 0:
		return 1.0 * sum / count
	return -1

def convertBatch(n):
	return [convert("%s.txt" % i) for i in range(n)]

def listToDict(ls):
	out = {}
	for i in range(len(ls)):
		if ls[i] == -1:
			continue
		out[i] = ls[i]
	return out
