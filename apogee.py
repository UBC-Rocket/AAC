#! usr/bin/python

print("apogee.py init")

import numpy as np
import matplotlib.pyplot as plt

# arrays and matrices
pars = np.zeros(4)
drvs = np.zeros(4)
wres = np.zeros(4)
pstp = np.zeros(4)
dstb = np.zeros(4)
wgtm = np.matrix(np.identity(4))
covm = np.matrix(np.identity(4))

# initial parameter values
# pars[0] = 9140.3  # amax, feet
# pars[1] = 16.4997    # tmax, seconds
# pars[2] = 15.5183    # drag time scale, seconds, represents the time for rocket to hit terminal velocity assuming it is in free fall, nose down
# pars[3] = 32.2    # deceleration, ft/s^2

pars[0] = 9115.7 #*1.1 # amax, feet
pars[1] = 15.91  #*1.05  # tmax, seconds
pars[2] = 15.97    # drag time scale, seconds, represents the time for rocket to hit terminal velocity assuming it is in free fall, nose down
pars[3] = 40.81   # deceleration, ft/s^2
# deploy = False


# initial covariances
covm[0,0] = 10.**2
covm[1,1] = 1.**2
covm[2,2] = 1.**2
covm[3,3] = (1.0)**2

# disturbance covariance per time step
hz = 20.0
dstb[0] = (1./hz)**2 * 0
dstb[1] = (1./hz)**2 * 0
dstb[2] = (1./hz)**2 * 0
dstb[3] = (1./hz)**2 * 0

# altimeter measurement variance (error squared)
altVar = 1.0

# start time for Kalman filter
tstart = 0.0

# altitude function
def altFunc(time):
    amax  = pars[0]
    tmax  = pars[1]
    tdrag = pars[2]
    decel = pars[3]
    altitude  = amax + decel * tdrag*tdrag * np.log(np.cos((time-tmax)/tdrag))
    return altitude

    # #Need to assign them to some algebraic function as a function of time
    # altDDiff = 0
    # timeDDiff = 0

    # #change altitude for prediction
    # if(deploy == True):
    #     amax = amax-altDDiff
    #     tmax = tmax-timeDDiff

# altitude derivatives function
def altDerivs(time):
    global drvs
    # amax  = pars[0]
    tmax  = pars[1]
    tdrag = pars[2]
    decel = pars[3]
    # altitude  = amax + decel * tdrag*tdrag * np.log(np.cos((time-tmax)/tdrag))
    drvs[0] = 1.0
    drvs[1] = decel * tdrag * np.tan((time-tmax)/tdrag)
    t1 = decel * tdrag*2.0  * np.log(np.cos((time-tmax)/tdrag))
    t2 = decel * (time-tmax) * np.tan((time-tmax)/tdrag)
    drvs[2] = t1+t2
    drvs[3] = tdrag * tdrag * np.log(np.cos((time-tmax)/tdrag))
    return


#Function to check if difference in altitude is suitable for deployment
# def deployCheck(time):
#     pass

def kalmanStep(ktime):
    global covm, wgtm, drvs, wres, ptable, etable, pstp, pars, pred, resd
    
    # current time and altitude
    time = dtable[ktime,0]
    alt  = dtable[ktime,2]

    # before start time, or after tmax, just copy info
    if time < tstart or time > pars[1]:
        pred[ktime] = altFunc(time)
        resd[ktime] = 0.0
        for j in range(4):
            ptable[ktime,j] = pars[j]
            etable[ktime,j] = np.sqrt(covm[j,j])
        return
    
    #Check for potential deployment
    # deployCheck()

    # predicted altitude
    altPred = altFunc(time)
    pred[ktime] = altPred
    # residual
    res = alt - altPred
    resd[ktime] = res
    # derivatives
    altDerivs(time)
    # weighted residuals array
    wres = res * drvs / altVar

    # degrade covariance matrix
    for i in range(4):
        covm[i,i] += dstb[i]
    # invert into weight matrix
    wgtm = covm.I
    
    # add information from new measurement
    for i in range(4):
        for j in range(4):
            wgtm[i,j] += drvs[i]*drvs[j] / altVar
    
    # invert into covariance
    covm = wgtm.I
    
    # find parameter steps
    pstp = covm.dot(wres)
    
    # update parameters, store info
    for j in range(4):
        pars[j] += pstp[0,j]
        ptable[ktime,j] = pars[j]
        etable[ktime,j] = np.sqrt(covm[j,j])
    
    return
    

# read data into table (columns are time, quantum-1, quantum-2)
dtable = np.loadtxt("formatted_data2.csv", delimiter=",")
#print table, "= table"

# number of time points
ntimes = dtable.shape[0]

# predictions and residuals
pred = np.zeros(ntimes)
resd = np.zeros(ntimes)

# parameters and errors tables
ptable = np.zeros((ntimes,4))
etable = np.zeros((ntimes,4))


# run the Kalman filter
for ktime in range(ntimes):
    kalmanStep(ktime)


plt.plot(dtable[:,0], dtable[:,2],"-")
plt.plot(dtable[:,0], pred[:],"-")
plt.errorbar(dtable[:,0], ptable[:,0], etable[:,0])
plt.title("altitude vs time")
#plt.ylim(-10.,10.)
plt.grid()
plt.show()

# print(ptable[-1,:])


# plt.plot(dtable[:,0], resd,"-")
# plt.title("resd vs time")
# plt.ylim(-100.,100.)
# plt.grid()
# plt.show()


# plt.errorbar(dtable[:,0], ptable[:,0], etable[:,0])
# plt.title("amax vs time")
# #plt.ylim(-5.,15.)
# plt.grid()
# plt.show()

# plt.errorbar(dtable[:,0], ptable[:,1], etable[:,1])
# plt.title("tmax vs time")
# #plt.ylim(-5.,15.)
# plt.grid()
# plt.show()

plt.errorbar(dtable[:,0], ptable[:,3], etable[:,3])
plt.title("decel vs time")
#plt.ylim(-5.,15.)
plt.grid()
plt.show()

plt.errorbar(dtable[:,0], ptable[:,2], etable[:,2])
plt.title("tdrag vs time")
#plt.ylim(-5.,15.)
plt.grid()
plt.show()


print("apogee.py done")