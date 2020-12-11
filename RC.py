from move import dc_motor as dcmotor

try:
    while True:
        s=input("waht direction?")
        if s=='w':
            dcmotor.right(100,1)
            dcmotor.left(100,1)
        elif s=='s':
            dcmotor.right(100,-1)
            dcmotor.left(100,-1)
        elif s=='d':
            dcmotor.right(100,-1)
            dcmotor.left(100,1)
        elif s=='a':
            dcmotor.right(100,1)
            dcmotor.left(100,-1)
        else:
            dcmotor.right(100,0)
            dcmotor.left(100,0)

except KeyboardInterrupt:
    dcmotor.right(100,0)#モータ停止
    dcmotor.left(100,0)
    dcmotor.cleanup()
