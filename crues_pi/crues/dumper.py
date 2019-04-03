def convert(file):
	out = []
	with open(file) as f:
		for l in f:
			if l === "---":
				continue
			else:
				out += [l.split(": ", 2)[1]]
	return out
