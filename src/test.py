a = 1
# aa = 2
def b():
    global a, \
        aa
    print(a)
    # print(aa)
    a=3
    aa=4
    print(a)
    print(aa)
def c():
    global a, \
        aa
    print(a)
    print(aa)
b()
c()