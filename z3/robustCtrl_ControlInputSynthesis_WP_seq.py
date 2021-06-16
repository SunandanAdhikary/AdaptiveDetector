# u altered : Control signal is neither computed nor communicated during drop
# y altered : Sensor data is not communicated. Considered as 0. Residue is considered as 0
# U delayed : U[k]=-KXhat[k-1] Control signal is calculated based on previous estimation 

# This code synthesizes robust controller gain and estimator gain to bring th system within
# preferable operating region under attack

import os
import numpy as np
import errno
############################## inputs ############################
modelName = "trajectory"

################## Patterns ###################

patternList = [1]

#########################################
initAttackLen = 1
offset = 0
start = 0
isSat = 0
innerCircleDepth = 0.2 
isDelayed = 0
yAffected = 0
####################################################################
if modelName == "tempControl":
    modelName= "tempControl"
    A= np.matrix('0.94648514795348381856143760160194 0.0018971440127071483982418298452899;0 0.000000000013887943864964020896356969649573')
    B= np.matrix('-0.046752721484125708828472056666214;-0.99999999998611222018496391683584')
    C= np.matrix('1 0')
    D= np.matrix('0')
    Gain= np.matrix('-0.3712408426327057364702000086254 -0.0007441187601164687840174516431091')
    L= np.matrix('0.60711740001928504728567759229918;0.39288259998275032458536770718638')
    safex = [30,30]
    tolerance = [1,1]
    th = 0.00001
elif modelName == "trajectory":
	A= np.matrix('1.0000    0.1000;0    1.0000')
	B= np.matrix('0.0050;0.1000')
	C= np.matrix('1 0')
	D= np.matrix('0')
	Gain= np.matrix('16.0302    5.6622')  # settling time around 10
	L = np.matrix('0.9902;0.9892')
	safex = [25,30]
	tolerance = [1,1]
	th = 4.35  #2
	sensorRange = [30]
	actuatorRange = [36]
	innerCircleDepth= 0.3 #0.2 #new
	nonatk_cov= 12.6041
	fromdepth= 0.7
elif modelName == "trajectory_pajic":# model from pajic's sporadic MAC CDC paper
	A= np.matrix('1.0000    0.1000;0    1.0000')
	B= np.matrix('0.0001;0.01')             
	C= np.matrix('1 0')
	D= np.matrix('0')
	Gain= np.matrix('16.0302    5.6622')  # settling time around 10
	L = np.matrix('0.6180 0.0011;0.0011 0.6180')
	safex = [0.025,0.025]
	tolerance = [0.1,0.1]
	th = 0.035
	fromdepth= 0.7
elif modelName == "esp":
	A= np.matrix('0.4450 -0.0458;1.2939 0.4402')
	B= np.matrix('0.0550;4.5607')
	C= np.matrix('0 1')
	D= np.matrix('0')
	# Gain= np.matrix('-0.0987 0.1420')
	Gain= np.matrix('0.2826    0.0960')
	L= np.matrix('-0.0390;0.4339')
	safex = [1,2]
	tolerance = [0.1,0.1]
	th = 4.35
	sensorRange = [2.5]
	actuatorRange = [0.8125]
	innerCircleDepth=0.1
	nonatk_cov= 0.1169
	fromdepth= 0.9
# elif modelName == "esp":
#     A= np.matrix(' 0.6278   -0.0259;0.4644    0.7071')
#     B= np.matrix(' 0.1246   -0.00000028;    3.2763    0.000016')
#     C= np.matrix('0    1.0000;-338.7813    1.1293')
#     D= np.matrix('0         0;  169.3907         0')
#     Gain= np.matrix('0.1365    0.1977;-0.0000    0.0000')
#     L= np.matrix('-0.000000000027085 -0.000000000636120;0.000000000336712 0.0000000055631')
#     safex = [1,2]
#     tolerance = [0.1,0.1]
#     th = 0.8
#     sensorRange = [2.5]
#     actuatorRange = [15]
elif modelName == "powersystem":
    A= np.matrix('0.66 0.53;-0.53 0.13')
    B= np.matrix('0.34;0.53')
    C= np.matrix('1 0;0 1')
    D= np.matrix('0;0')
    Gain= np.matrix('0.0556 0.3306')
    L= np.matrix('0.36 0.27;  -0.31 0.08')
    safex = [0.1,0.05]
    tolerance = [0.001,0.0001]
    # initialRange = np.matrix('0 0.35;-4 4')
    th = 0.03
elif modelName == "plant":
    A= np.matrix('2.6221    0.3197    1.8335   -1.0664; -0.2381    0.1872   -0.1361    0.2017; 0.1612    0.7888    0.2859    0.6064;-0.1035    0.7641    0.0886    0.7360')
    B= np.matrix('0.4654   -1.5495; 1.3138    0.0851; 2.0549   -0.6730; 2.0227   -0.1597')
    C= np.matrix('1     0     1    -1;0     1     0     0')
    D= np.matrix('0 0;0 0')
    Gain= np.matrix('-0.2580    0.3159   -0.1087    0.3982; -1.6195   -0.1314   -1.1232    0.7073')
    L= np.matrix('2.4701   -0.0499; -0.2144    0.0224; 0.2327    0.0946; -0.0192    0.1004')
    safex = [0.01,0.01,0.01,0.01]
    tolerance = [0.0001,0.0001,0.0001,0.0001]
    th = 0.0001
elif modelName == "powergrid":
    A= np.matrix('-1 -3;3 -5')
    B= np.matrix('2 -1;1 0')
    C= np.matrix('0.8 2.4;1.6 0.8')
    D= np.matrix('0 0; 0 0')
    Gain= np.matrix('2.9846   -4.9827;6.9635   -6.9599')
    L= np.matrix('-1.1751   -0.1412;-2.6599    2.2549')
    safex = [0.1,0.2]
    tolerance = [0.001,0.001]
    th = 0.01

################## creating the path to save results #################
path="results/robust_Ctrl/"+modelName+"/"
try:
    os.makedirs(path)
except OSError as err:
    if err.errno!= errno.EEXIST:
        raise
#####################################################################
u_count=B.shape[1]
x_count=A.shape[1]
y_count=C.shape[0]
depths = [1,innerCircleDepth]#new
######### Configure which sensor/control input to attack ##########
u_attack_map = np.zeros(u_count,dtype=float)
y_attack_map = np.zeros(y_count,dtype=float)
u_attack_map[0] = 0
y_attack_map[0] = 0



######### Configure which sensor/control input to attack ##########
initialRange = np.zeros(shape=(x_count,2), dtype=float)
for i in range(x_count):
    initialRange[i,0] = (-1)*safex[i]*innerCircleDepth
    initialRange[i,1] = safex[i]*innerCircleDepth

print(set(patternList))
#Compute pattern length
for pattern in set(patternList):
    print("Running for "+str(pattern))
    patternLen=len(str(pattern))

    #Compute pattern array
    patternArray = np.zeros((patternLen), dtype=int)
    patternTemp = pattern
    i = patternLen-1
    while patternTemp>0:
        patternArray[i] = patternTemp%10
        patternTemp=patternTemp//10
        i = i-1

    f0 = open(path+modelName+"_robustC.z3result", "w+")
    f0.write("0")
    f0.close()

    print("Finding robust control gains for "+str(modelName))
    isSat = 0
    attackLen = initAttackLen
    
    while isSat == 0:  
        index = start
        while (index<patternLen) and (isSat == 0):
            # create drop pattern
            K = index + attackLen + offset
            print("Trying for robust controller in "+str(K)+" steps.")
            forallvars="" #new
            antecedent=""
            consequent=""
            dropPattern = np.ones((K), dtype=int)
            j=0
            if pattern!=1:
                for i in range(K):
                    dropPattern[i] = patternArray[j]
                    j = j + 1
                    if j == patternLen:
                        j = 0
            print("drop pattern:")
            print(dropPattern)
            print("\n")
            fileName = modelName + "_robustCtrl_"+ str(K)+"step.py"
            f = open(path+fileName, "w+")
            f.write("from z3 import *\n")
            f.write("import math\n")
            f.write("import numpy as np\n\n")
            f.write("s = Solver()\n")
            # f.write("set_option(precision=40)\n")
            f.write("set_option(rational_to_decimal=True)\n")
            # declaring variables
            for varcount in range(1,x_count+1):
                f.write("xabs"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("zabs"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")

                f.write("x"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("z"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
            for varcount in range(1,y_count+1):
                f.write("y"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("attackOnY"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("r"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
            for varcount in range(1,u_count+1):
                f.write("u"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("uattacked"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
                f.write("attackOnU"+str(varcount)+" = np.zeros("+str(K+1)+", dtype=float)\n")
               
            f.write("r = np.zeros("+str(K+1)+", dtype=float)\n\n")
            f.write("RGain={}\n") #new
            # f.write("RObs={}\n") #new
            for ii in range(K+1):
                f.write("RGain["+str(ii+1)+"] = np.zeros(("+str(u_count)+","+str(x_count)+"), dtype=float)\n")# new block
                # f.write("RObs["+str(ii+1)+"] = np.zeros(("+str(x_count)+","+str(y_count)+"), dtype=float)\n")# new block
            for i in range(K+1):
                decl="#declarations\n"#new
                for varcount in range(1,y_count+1):
                    decl+="y"+str(varcount)+"_"+str(i)+" = Real('y"+str(varcount)+"_"+str(i)+"')\n"
                    decl+="r"+str(varcount)+"_"+str(i)+" = Real('r"+str(varcount)+"_"+str(i)+"')\n"
                    decl+="rabs"+str(varcount)+"_"+str(i)+" = Real('rabs"+str(varcount)+"_"+str(i)+"')\n"
    

                for varcount in range(1,x_count+1):
                    decl+="x"+str(varcount)+"_"+str(i)+" = Real('x"+str(varcount)+"_"+str(i)+"')\n"
                    decl+="z"+str(varcount)+"_"+str(i)+" = Real('z"+str(varcount)+"_"+str(i)+"')\n"

                    decl+="xabs"+str(varcount)+"_"+str(i)+" = Real('xabs"+str(varcount)+"_"+str(i)+"')\n"
                    for varcount2 in range(1,u_count+1):
                        decl+="RGain"+str(varcount2)+str(varcount)+"_"+str(i)+" = Real('RGain"+str(varcount2)+str(varcount)+"_"+str(i)+"')\n"#new
                    decl+="zabs"+str(varcount)+"_"+str(i)+" = Real('zabs"+str(varcount)+"_"+str(i)+"')\n"
                
                for varcount in range(1,u_count+1):
                    decl+="u"+str(varcount)+"_"+str(i)+" = Real('u"+str(varcount)+"_"+str(i)+"')\n"
                    decl+="uattacked"+str(varcount)+"_"+str(i)+" = Real('uattacked"+str(varcount)+"_"+str(i)+"')\n"

                decl+="r_"+str(i)+" = Real('r_"+str(i)+"')\n"
                
                f.write(decl)
            #initial range declaration of variables    
            f.write("\n#Init\n")
            for varcount in range(1,x_count+1):
                antecedent+="Or(x"+str(varcount)+"_0 == "+str(fromdepth*safex[varcount-1])+",x"+str(varcount)+"_0 == "+str(-fromdepth*safex[varcount-1])+"),\n"
                forallvars+="x"+str(varcount)+"_0,"
                antecedent+="And(z"+str(varcount)+"_0 <= "+str(fromdepth*safex[varcount-1])+",z"+str(varcount)+"_0 >= "+str(-fromdepth*safex[varcount-1])+"),\n"
                forallvars+="z"+str(varcount)+"_0,"
            f.write("\n")

            for i in range(K):
                for varcount1 in range(1,u_count+1):
                    consequent+="u"+str(varcount1)+"_"+str(i)+" == "
                    for varcount2 in range(1,x_count+1):
                        consequent+=" - (("+str(Gain[varcount1-1,varcount2-1])+"+RGain"+str(varcount1)+str(varcount2)+"_"+str(i)+")*z"+str(varcount2)+"_"+str(i)+")"                        
                    consequent +="\n"

                for varcount1 in range(1,u_count+1):
                    f.write("attackOnU"+str(varcount1)+"_"+str(i)+" = Real('attackOnU"+str(varcount1)+"_"+str(i)+"')\n")
                    forallvars+="attackOnU"+str(varcount1)+"_"+str(i)+","
                    consequent+="uattacked"+str(varcount1)+"_"+str(i)+" == u"+str(varcount1)+"_"+str(i)+"+ ("+str(u_attack_map[varcount1-1])+"*attackOnU"+str(varcount1)+"_"+str(i)+")\n"
                    # antecedent+="And(attackOnU"+str(varcount1)+"_"+str(i)+">-"+str(actuatorRange[varcount1-1])+",attackOnU"+str(varcount1)+"_"+str(i)+"<"+str(actuatorRange[varcount1-1])+"),\n"
                    antecedent+="attackOnU"+str(varcount1)+"_"+str(i)+"==0.00,\n"
                    antecedent+="(r"+str(varcount1)+"_"+str(i)+"*r"+str(varcount1)+"_"+str(i)+"+"
                    
                antecedent= antecedent[:len(antecedent)-1]
                antecedent+="<="+str(nonatk_cov)+"*"+str(th)+"),\n"
            # chi sq res

                for varcount1 in range(1,u_count+1):
                    if u_attack_map[varcount1-1] and actuatorRange[varcount1-1]!=0:
                        antecedent+="And(u"+str(varcount1)+"_"+str(i)+">-"+str(actuatorRange[varcount1-1])+",u"+str(varcount1)+"_"+str(i)+"<"+str(actuatorRange[varcount1-1])+"),\n"
                        antecedent+="And(uattacked"+str(varcount1)+"_"+str(i)+">-"+str(actuatorRange[varcount1-1])+",uattacked"+str(varcount1)+"_"+str(i)+"<"+str(actuatorRange[varcount1-1])+"),\n"

                for varcount1 in range(1,y_count+1):
                    f.write("attackOnY"+str(varcount1)+"_"+str(i)+" = Real('attackOnY"+str(varcount1)+"_"+str(i)+"')\n")
                    consequent+="y"+str(varcount1)+"_"+str(i)+" == ("+str(y_attack_map[varcount1-1])+"*attackOnY"+str(varcount1)+"_"+str(i)+")"
                    forallvars+="attackOnY"+str(varcount1)+"_"+str(i)+","
                    for varcount2 in range(1,x_count+1):
                        consequent+=" + ("+str(C[varcount1-1,varcount2-1])+"*x"+str(varcount2)+"_"+str(i)+")"
                    for varcount3 in range(1,u_count+1):
                        consequent+=" + ("+str(D[varcount1-1,varcount3-1])+"*uattacked"+str(varcount3)+"_"+str(i)+")" 
                    consequent+="\n"
                    # antecedent+="And(attackOnY"+str(varcount1)+"_"+str(i)+">-"+str(sensorRange[varcount1-1])+",attackOnY"+str(varcount1)+"_"+str(i)+"<"+str(sensorRange[varcount1-1])+"),\n"
                    antecedent+="attackOnY"+str(varcount1)+"_"+str(i)+"== 0.00,\n"


                for varcount1 in range(1,y_count+1):
                    if y_attack_map[varcount1-1] and sensorRange[varcount1-1]!=0:
                        antecedent+="And(y"+str(varcount1)+"_"+str(i)+">-"+str(sensorRange[varcount1-1])+",y"+str(varcount1)+"_"+str(i)+"<"+str(sensorRange[varcount1-1])+"),\n"

                for varcount1 in range(1,y_count+1):
                    consequent+="r"+str(varcount1)+"_"+str(i)+" == y"+str(varcount1)+"_"+str(i)
                    for varcount2 in range(1,x_count+1):
                        consequent+=" - ("+str(C[varcount1-1,varcount2-1])+"*z"+str(varcount2)+"_"+str(i)+")"
                    for varcount3 in range(1,u_count+1):       
                        consequent+=" - ("+str(D[varcount1-1,varcount3-1])+"*u"+str(varcount3)+"_"+str(i)+")"   
                    consequent+="\n"

                expr_x=""
                expr_xabs=""
                expr_z=""
                expr_zabs=""
                for varcount1 in range(1,x_count+1):
                    expr_x+="x"+str(varcount1)+"_"+str(i+1)+" == "                
                    expr_z+="z"+str(varcount1)+"_"+str(i+1)+" == "
                    for varcount2 in range(1,x_count+1):
                        expr_x+=" ("+str(A[varcount1-1,varcount2-1])+"*x"+str(varcount2)+"_"+str(i)+") +"
                        expr_z+=" ("+str(A[varcount1-1,varcount2-1])+"*z"+str(varcount2)+"_"+str(i)+") +"
                    for varcount3 in range(1,u_count+1):
                        expr_z+=" ("+str(B[varcount1-1,varcount3-1])+"*u"+str(varcount3)+"_"+str(i)+") +"
                        expr_x+=" ("+str(B[varcount1-1,varcount3-1])+"*uattacked"+str(varcount3)+"_"+str(i)+") +"
                    for varcount4 in range(1,y_count+1):
                        expr_z+=" ("+str(L[varcount1-1,varcount4-1])+"*r"+str(varcount4)+"_"+str(i)+") +"
                    
                    expr_xabs+="xabs"+str(varcount1)+"_"+str(i+1)+" == If(x"+str(varcount1)+"_"+str(i+1)+"<0,(-1)*x"+str(varcount1)+"_"+str(i+1)+",x"+str(varcount1)+"_"+str(i+1)+")\n"

                    expr_zabs+="zabs"+str(varcount1)+"_"+str(i+1)+" == If(z"+str(varcount1)+"_"+str(i+1)+"<0,(-1)*z"+str(varcount1)+"_"+str(i+1)+",z"+str(varcount1)+"_"+str(i+1)+")\n"

                    expr_x=expr_x[:len(expr_x)-1] 
                    expr_x = expr_x + "\n"
                    expr_z=expr_z[:len(expr_z)-1] 
                    expr_z = expr_z + "\n"            

                consequent+=expr_x+expr_z+expr_xabs+expr_zabs            
        
            for varcount in range(1,x_count+1):
                for i in range(K-1):
                    consequent+="xabs"+str(varcount)+"_"+str(i+1)+" < "+str(safex[varcount-1])+",\n"
                    consequent+="zabs"+str(varcount)+"_"+str(i+1)+" < "+str(safex[varcount-1])+",\n"
            for varcount in range(1,x_count+1):
                consequent+="xabs"+str(varcount)+"_"+str(K)+" < "+str((1-(K)*(1-innerCircleDepth)/K)*safex[varcount-1])+",\n"
                consequent+="zabs"+str(varcount)+"_"+str(K)+" < "+str((1-(K)*(1-innerCircleDepth)/K)*safex[varcount-1])+",\n"
            #format consequent
            antecedent = antecedent[:len(antecedent)-1]
            consequent = consequent[:len(consequent)-2]

            noOfStatements =len(consequent.splitlines())
            statements = consequent.splitlines()
            # print("no of statement sin consequent\n")
            # print(str(noOfStatements)+"\n")
            # print("antecedent in original\n")
            # print(antecedent+"\n")
            # print("consequent in original\n")
            # print(consequent+"\n")
            while noOfStatements>(x_count*2*K):
                temp1 = str(statements[0]).split("==")[0].rstrip().lstrip()
                temp2 = str(statements[0]).split("==")[1].rstrip().lstrip()
                new = "("+temp2+")"
                consequent=consequent.replace(str(statements[0])+"\n","")
                consequent=consequent.replace(temp1,new)
                antecedent = antecedent.replace(temp1,new)
                # print("\nafter replacing "+temp1+" by "+temp2+"\n")
                # print(consequent)
                noOfStatements =len(consequent.splitlines())
                statements = consequent.splitlines()
            
            forallvars=forallvars[:len(forallvars)-1]
            assertion="s.add(ForAll(["+forallvars+"],\nImplies(\nAnd(\n"+antecedent+"\n),\nAnd(\n"+consequent+"\n)  \n)\n))"     
            f.write(assertion)

            f.write("\nfsmt = open(\""+path+modelName+"_"+str(K)+".smt2\", \"w+\")\n")
            f.write("fsmt.write(s.sexpr())\n")
            f.write("fsmt.write(\"(check-smt)\")\n")
            # f.write("fsmt.write(\"(get-model)\")\n")
            f.write("fsmt.close()\n")
            f.write("\nif s.check() != sat:\n")
            f.write("\tprint(s.check())\n")
            f.write("\tisSat = 0\n")
            f.write("else:\n")
            f.write("\tprint(s.check())\n")
            f.write("\tisSat = 1\n")
            f.write("\tf0 = open(\""+path+modelName+"_robustC.z3result\", \"w+\")\n")
            f.write("\tf0.write(\"1\")\n")
            f.write("\tf0.close()\n") 
            f.write("\tm = s.model()\n")
            f.write("\tfor d in m.decls():\n")
            f.write("\t\tprint (\"%s = %s\" % (d.name(), m[d]))\n")
            
            # for varcount in range(0,u_count): 
            #     # Parsing robust gain values
            #     for varcount2 in range(0,x_count): # Parsing gains
            #         f.write("\t\tif \"RGain"+str(varcount+1)+str(varcount2+1)+"_\" in d.name():\n")
            #         f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #         f.write("\t\t\tx = str(m[d])\n")
            #         f.write("\t\t\tindex = x.find(\"?\")\n")
            #         f.write("\t\t\tif index != -1:\n")
            #         f.write("\t\t\t\ty = x[:-1]\n")
            #         f.write("\t\t\telse:\n")
            #         f.write("\t\t\t\ty = x\n")
            #         f.write("\t\t\tRGain[i]["+str(varcount)+","+str(varcount2)+"] ="+str(Gain[varcount,varcount2])+" + float(y)\n")#new block

            #     # Parsing u, attack on u and attacked u values from the z3 py output
            #     f.write("\t\tif \"attackOnU"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tattackOnU"+str(varcount)+"[i] = float(y)\n")

            #     f.write("\t\tif \"uattacked"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tuattacked"+str(varcount)+"[i] = float(y)\n")

            #     f.write("\t\tif \"u"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tu"+str(varcount)+"[i] = float(y)\n")
                
            # for varcount in range(1,x_count+1): # Parsing x, absolute x and z from z3 py output
                
            #     f.write("\t\tif \"z"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tz"+str(varcount)+"[i] = float(y)\n")

            #     f.write("\t\tif \"xabs"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\txabs"+str(varcount)+"[i] = float(y)\n")

            #     f.write("\t\tif \"x"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tx"+str(varcount)+"[i] = float(y)\n")
            # for varcount in range(1,y_count+1): # Parsing y, attack on y and r from z3 py output
            #     f.write("\t\tif \"attackOnY"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tattackOnY"+str(varcount)+"[i] = float(y)\n")

            #     f.write("\t\tif \"y"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\ty"+str(varcount)+"[i] = float(y)\n")
                
            #     f.write("\t\tif \"r"+str(varcount)+"_\" in d.name():\n")
            #     f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            #     f.write("\t\t\tx = str(m[d])\n")
            #     f.write("\t\t\tindex = x.find(\"?\")\n")
            #     f.write("\t\t\tif index != -1:\n")
            #     f.write("\t\t\t\ty = x[:-1]\n")
            #     f.write("\t\t\telse:\n")
            #     f.write("\t\t\t\ty = x\n")
            #     f.write("\t\t\tr"+str(varcount)+"[i] = float(y)\n")
            
            # f.write("\t\tif \"r_\" in d.name():\n")
            # f.write("\t\t\ti = int(d.name().split('_')[1])\n")
            # f.write("\t\t\tx = str(m[d])\n")
            # f.write("\t\t\tindex = x.find(\"?\")\n")
            # f.write("\t\t\tif index != -1:\n")
            # f.write("\t\t\t\ty = x[:-1]\n")
            # f.write("\t\t\telse:\n")
            # f.write("\t\t\t\ty = x\n")
            # f.write("\t\t\tr[i] = float(y)\n")

            # # Printing the execution sequence
            # f.write("\tfor i in range({0}):\n".format(K))
            # for varcount in  range(1, x_count+1):
            #     f.write("\t\tprint(\"x"+str(varcount)+"_{0}={1}\".format(i,x"+str(varcount)+"[i]))\n") 
            # for varcount in  range(1, x_count+1):
            #     f.write("\t\tprint(\"z"+str(varcount)+"_{0}={1}\".format(i,z"+str(varcount)+"[i]))\n") 
            # for varcount in  range(1, x_count+1):
            #     f.write("\t\tprint(\"xabs"+str(varcount)+"_{0}={1}\".format(i,xabs"+str(varcount)+"[i]))\n") 
            # for varcount in  range(1, u_count+1):
            #     f.write("\t\tprint(\"attackOnU"+str(varcount)+"_{0}={1}\".format(i,attackOnU"+str(varcount)+"[i]))\n")  
            # for varcount in  range(1, u_count+1):
            #     f.write("\t\tprint(\"u"+str(varcount)+"_{0}={1}\".format(i,u"+str(varcount)+"[i]))\n")  
            # for varcount in  range(1, u_count+1):
            #     f.write("\t\tprint(\"uattacked"+str(varcount)+"_{0}={1}\".format(i,uattacked"+str(varcount)+"[i]))\n")
            # for varcount in  range(1, y_count+1):
            #     f.write("\t\tprint(\"attackOnY"+str(varcount)+"_{0}={1}\".format(i,attackOnY"+str(varcount)+"[i]))\n") 
            # for varcount in  range(1, y_count+1):
            #     f.write("\t\tprint(\"y"+str(varcount)+"_{0}={1}\".format(i,y"+str(varcount)+"[i]))\n")
            # for varcount in  range(1, y_count+1):
            #     f.write("\t\tprint(\"r"+str(varcount)+"_{0}={1}\".format(i,r"+str(varcount)+"[i]))\n")
            # f.write("\t\tprint(\"r_{0}={1}\".format(i,r[i]))\n")
            
            # # Printing robust gain values
            # f.write("\t\tprint(\"RGain[{0}]\".format(i))\n")#new
              
            # # Printing attack vectors

            # for varcount in  range(1, u_count+1):
            #     f.write("print(\"attack on control signal component {0}\")\n".format(varcount))
            #     f.write("print(attackOnU{0})\n".format(varcount))
            # for varcount in  range(1, y_count+1):
            #     f.write("print(\"attack on sensor {0}\")\n".format(varcount))
            #     f.write("print(attackOnY{0})\n".format(varcount))
        

            f.write("if isSat==1:\n")
            f.write("\tf0 = open(\""+path+modelName+"_robustC.z3result\", \"w+\")\n")
            f.write("\tf0.write(\"1\")\n")
            f.write("\tf0.close()\n")
            f.close()

            isSat=1

            os.system("python "+path+fileName+">"+path+fileName+".z3out")
            f0 = open(path+modelName+"_robustC.z3result", "r")
            if f0.mode == 'r':
                content = f0.read()
                isSat = int(content)
            f0.close()
            if isSat==1:
                print("Gain found")                
                # os.remove(path+fileName)
                # os.remove(path+fileName+".z3out")
                print("go to "+path+fileName+".z3out \n")
            else:
                print("Gain not found")
                # os.system("rm -rf "+path+fileName+" "+path+fileName+".z3out")
                attackLen = attackLen + 1#new
