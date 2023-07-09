import sys
from string import Template

swarmSize=3
if len(sys.argv)==0:
    print(f'Default swarm size {swarmSize}')
else:
    swarmSize=int(sys.argv[1])
    
print('''
{
    "world_name": "empty",
    "drones": [''')

temp = Template('''
        {
            "model_type": "quadrotor_base",
            "model_name": "drone0",
            "xyz": [
                $x,
                $y,
                $z
            ],
            "rpy": [
                0,
                0,
                0.0
            ],
            "flight_time": 60
        }$sep''')
currX=0
currY=0
currZ=0
sep=','
for i in range(swarmSize):
    minV=min(currX, currY, currZ)
    if currX==minV:
        currX+=1
    elif currY==minV:
        currY+=1
    elif currZ==minV:
        currZ+=1
    if (i==swarmSize-1):
        sep=''
    print(temp.substitute({'x': str(currX), 'y': str(currY), 'z': str(currZ), 'sep': sep}))
    
print('''
    ]
}
''')