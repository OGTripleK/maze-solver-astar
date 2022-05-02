from queue import PriorityQueue
from pyamaze import maze,textLabel,agent,COLOR
from math import sqrt
from easygui import *
from timeit import timeit
import time
from tqdm import tqdm
print("-------------------------------------------\nMaze Solver All-in-One V5.0 Final Release(EXE EDITION)\n-------------------------------------------")
print("Changelog")
print('--------------------------------------------')
print("V5.0-Final Release (EXE EDITION)\n - Add Time Compare in Compare Result")
print('--------------------------------------------')
print("V4.0\n - Compare Result (Run maze solver without GUI for multiple times and display result)")
print('--------------------------------------------')
print("V3.0\n - Customizable Maze with GUI on start\n - Compare Both to compare Manhattan and Euclidean")
print('--------------------------------------------')
print("V2.0\n - Astar with Manhattan Update\n - Euclidean Distance Now Available")
print('--------------------------------------------')
print("V1.0\n - Initial release\n - User Control")
print('--------------------------------------------')
print("Created By Parsawit Deshwattanatham , Chitiphat Jaroonwanit, Petch Vessayapiroon")
# I LIKE IT THIS WAY, ITS LOOK BETTER ALRIGHT?
if __name__=='__main__':
    running = True
else:
    running = False

Compare = False
Result = False
text = "Enter the following details"
text_box = "Choose Distance Method"
# window title
title = "Maze Option"

# list of multiple inputs
button_list = ["Manhattan Distance", "Euclidean Distance", "Compare Both", "Compare Result"]
input_list = ["X Length", "Y Length", "Loop Size (0-100)", "Visualization?"]
input_list_result = ["X Length", "Y Length", "Loop Size (0-100)", "Compare Result How Many Times?"]

# list of default text
default_list = ["12", "12", "50", "Yes"]
default_list_result = ["12", "12", "50", "100"]

# creating a box
method = buttonbox(text_box, title, button_list)
if method == "Compare Result":
    Result = True
    output = multenterbox(text, title, input_list_result, default_list_result)
else:
    output = multenterbox(text, title, input_list, default_list)

x_ans = int(output[0])
y_ans = int(output[1])
loop_ans = int(output[2])
if Result == True:
    visual_ans = 0
    result_val = int(output[3])
else:
    visual_ans = output[3]
    #visual convert lower
    visual_ans = visual_ans.lower()
    #visual convert int
    if visual_ans == "yes":
        visual_ans = 50
    else:
        visual_ans = 0
# Comapre Convert
if method == "Compare Both":
    Compare = True

    

#Manhattan Distance / Euclidean / Compare:Manhattan
def h(a, b):
    x1, y1 = a
    x2, y2 = b
    if method == "Manhattan Distance":
        return (abs(x1 - x2) + abs(y1 - y2))
    elif method == "Euclidean Distance":
        return sqrt((x1-x2)**2+(y1-y2)**2)
    else:
        return (abs(x1 - x2) + abs(y1 - y2))

#Euclidean 4 compare
def euc(a,b):
    x1, y1 = a
    x2, y2 = b
    return sqrt((x1-x2)**2+(y1-y2)**2)

#A* Algorithm
def aStar(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    #setup queue
    open = PriorityQueue()
    #Put in queue(fcost from start, heuristic cost, cell value)
    open.put((h(start, m._goal), h(start, m._goal), start))
    #set found path as dict
    aPath = {}
    #float("inf") unbounded upper value for comparison
    g_score = {row: float("inf") for row in m.grid}
    #set g(n) = 0 from start
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    #f(n) score = distance from start to goal
    f_score[start] = h(start, m._goal)
    #Pathfinding setup
    searchPath=[start]
    while not open.empty():
        #Specified get cellvalue which [2] is from queue 
        currCell = open.get()[2]
        searchPath.append(currCell)
        #if reach goal break loop
        if currCell == m._goal:
            break        
        # GOOOOOOOOOO CHECK ALL DIRECTIONNNN
        for d in 'ESNW':
            #4 direction check for 1 = free no wall
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])
                #g_score for current cell
                temp_g_score = g_score[currCell] + 1
                #f_score is new g_score + heuristic cost of child cell
                temp_f_score = temp_g_score + h(childCell, m._goal)

                #Check if NEW f_score is more than previous f_score
                if temp_f_score < f_score[childCell]:
                    #if true then it will update g score and f score of the child cell
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + h(childCell, m._goal)
                    #Put up in queue (f score of childcell, h score of child cell, then childcell)
                    open.put((f_score[childCell], h(childCell, m._goal), childCell))

    fwdPath={}
    cell=m._goal
    #Make it reverse so it can be written on dictionary as multiple value and can be use as reverse path
    while cell!=start:
        fwdPath[aPath[cell]]=cell
        cell=aPath[cell]
    return searchPath,aPath,fwdPath

def aStar2(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((euc(start, m._goal), euc(start, m._goal), start))
    aPath2 = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = euc(start, m._goal)
    searchPath2=[start]
    while not open.empty():
        currCell = open.get()[2]
        searchPath2.append(currCell)
        if currCell == m._goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])

                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + euc(childCell, m._goal)

                if temp_f_score < f_score[childCell]:   
                    aPath2[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + euc(childCell, m._goal)
                    open.put((f_score[childCell], euc(childCell, m._goal), childCell))


    fwdPath2={}
    cell=m._goal
    while cell!=start:
        fwdPath2[aPath2[cell]]=cell
        cell=aPath2[cell]
    return searchPath2,aPath2,fwdPath2
for o in range(30):
    print(" ")
if running == True:
    if Compare == False and Result == False:
        m=maze(x_ans,y_ans)
        #saved maze file name: saved_maze_loop75.csv, saved_maze_loop100.csv
        m.CreateMaze(loopPercent=loop_ans)

        searchPath,aPath,fwdPath = aStar(m)
        #Pathfinding Visualization
        a=agent(m,footprints=True,color=COLOR.green,filled=True)
        #Goal Colored Box
        b=agent(m,1,1,footprints=True,color=COLOR.yellow,filled=True,goal=(m.rows,m.cols))
        #Dot line of pathfinding
        c=agent(m,footprints=True,color=COLOR.red)

        m.tracePath({a:searchPath},delay=visual_ans)
        m.tracePath({b:aPath},delay=visual_ans)
        m.tracePath({c:fwdPath},delay=visual_ans)

        l=textLabel(m,'Exit Path Length:',len(fwdPath)+1)
        l=textLabel(m,'Total Path Search:',len(searchPath))
        m.run()
    elif Compare == True and Result == False:
        m=maze(x_ans,y_ans)
        #saved maze file name: saved_maze_loop75.csv, saved_maze_loop100.csv
        m.CreateMaze(loopPercent=loop_ans)
        searchPath,aPath,fwdPath = aStar(m)
        searchPath2,aPath2,fwdPath2=aStar2(m)

        #Pathfinding Visualization
        a=agent(m,footprints=True,color=COLOR.green,filled=True)
        #Goal Colored Box
        b=agent(m,1,1,footprints=True,color=COLOR.yellow,filled=True,goal=(m.rows,m.cols))
        #Dot line of pathfinding
        c=agent(m,footprints=True,color=COLOR.red,filled=True)

        #Pathfinding Visualization
        d=agent(m,footprints=True,color=COLOR.blue)
        #Goal Colored Box
        e=agent(m,1,1,footprints=True,color=COLOR.red,goal=(m.rows,m.cols))
        #Dot line of pathfinding
        f=agent(m,footprints=True,color=COLOR.yellow)

        m.tracePath({a:searchPath},delay=visual_ans)
        m.tracePath({b:aPath},delay=visual_ans)
        m.tracePath({c:fwdPath},delay=visual_ans)

        m.tracePath({d:searchPath2},delay=visual_ans)
        m.tracePath({e:aPath2},delay=visual_ans)
        m.tracePath({f:fwdPath2},delay=visual_ans)

        l=textLabel(m,'Manhattan Shortest Path:',len(fwdPath)+1)
        l=textLabel(m,'Euclidean Shortest Path:',len(fwdPath2)+1)
        l=textLabel(m,'Manhattan Search Path:',len(searchPath)+1)
        l=textLabel(m,'Euclidean Search Path:',len(searchPath2)+1)

        t1=timeit(stmt='aStar(m)',number=100,globals=globals())
        t2=timeit(stmt='aStar2(m)',number=100,globals=globals())
        textLabel(m,'Manhattan Time Taken',t1)
        textLabel(m,'Euclidean Time Taken:',t2)
        m.run()
    else:
        f1, f2, f3 = 0, 0, 0
        s1, s2, s3 = 0, 0, 0
        t1, t2, t3 = 0, 0, 0
        print(f'Simulating {result_val} times')
        for i in tqdm(range(result_val)):
            m=maze(x_ans,y_ans)
            #saved maze file name: saved_maze_loop75.csv, saved_maze_loop100.csv
            m.CreateMaze(loopPercent=loop_ans)
            searchPath,aPath,fwdPath = aStar(m)
            searchPath2,aPath2,fwdPath2=aStar2(m)
            if len(fwdPath)==len(fwdPath2):
                f1+=1
            elif len(fwdPath)<len(fwdPath2):
                f2+=1
            else:
                f3+=1

            if len(searchPath)==len(searchPath2):
                s1+=1
            elif len(searchPath)<len(searchPath2):
                s2+=1
            else:
                s3+=1
            #Manhat
            man_counter=timeit(stmt='aStar(m)',number=100,globals=globals())
            #Eucli
            euc_counter=timeit(stmt='aStar2(m)',number=100,globals=globals())
            if man_counter == euc_counter:
                t1+=1
            if man_counter < euc_counter:
                t2+=1
            else:
                t3+=1
            
        print(f'\rSimulating {result_val} Result Completed', end = '')
        for o in range(5):
            print(" ")
        print(f'\n--------------------------------------------\nMaze Settings\n--------------------------------------------\nMaze Size(X,Y):({x_ans},{y_ans})\nLoop Size :{loop_ans}\n--------------------------------------------')
        print('Final Path Comparison Result') 
        print(f'Both have same Final Path length for {f1} times.')
        print(f'Manhattan has lesser Final Path length for {f2} times.')
        print(f'Euclidean has lesser Final Path length for {f3} times.')

        print('--------------------------------------------')

        print('Search Path Comparison Result')
        print(f'Both have same Search Path length for {s1} times.')
        print(f'Manhattan has lesser Search Path length for {s2} times.')
        print(f'Euclidean has lesser Search Path length for {s3} times.')
        print('--------------------------------------------')
        print('Time Taken Comparison Result')
        print(f'Both have same time taken for {t1} times.')
        print(f'Manhattan is faster than Euclidean for {t2} times.')
        print(f'Euclidean is faster than Manhattan for {t3} times.')
time.sleep(3600)
