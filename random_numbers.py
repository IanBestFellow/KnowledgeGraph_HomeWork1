import random
random.seed(123)
lst = []
for i in range(1000):
    a = random.randint(0,1000)
    b = random.randint(0,1000)
    lst.append([a,b])
print(lst)