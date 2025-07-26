import math
def cal_ak(k):
    ak = 4/3*k**3-k**2+2/3*k
    return ak

def cal_bk(k):
    m=math.floor( math.log2(k) )
    bk=8**m + cal_ak(k)-cal_ak(2**m)
    return bk

def t_inv_n(n):
    ''' n 必须是4的倍数'''
    k=int(n/4)
    
    return cal_bk(k)

print(t_inv_n(4))
print(t_inv_n(8))
print(t_inv_n(12))
print(t_inv_n(16))
print(t_inv_n(20))

print(t_inv_n(64))