flag=True
def true_hgt():
    global flag
    if (i>100):
        flag=False
    if (flag==True):
        hgt=9
    else:
        hgt=0
    print hgt
i=1
c=1

while True:
   i+=1
   c+=1
   if(c>500):
       i=0
   true_hgt()
   
