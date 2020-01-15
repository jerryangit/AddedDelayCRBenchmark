from operator import itemgetter
test = {}
test[1] = 10
test[8] = 15
test[4] = 5
test[2] = 50
print(test)
test2 = sorted(test.items(),key=itemgetter(1))
print(test2)
print(test2[0][0])
print(len(test2))

for pair in enumerate(test2):
    print(pair[1][0])


