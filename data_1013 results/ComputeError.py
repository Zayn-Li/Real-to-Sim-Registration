import numpy as np
import sys
import os
from plyfile import PlyData

def ComputeError(Observed_dir, Simu_dir):
    Observed_file = os.listdir(Observed_dir)
    Simu_file = os.listdir(Simu_dir)
    Observed_file.sort(key= lambda x:int(x[:x.index('.')]))
    Simu_file.sort(key= lambda x:int(x[x.index('_')+1:x.index('.')]))
    if len(Observed_file) != len(Simu_file):
        print("Please check the input files!")
        return -1
    else:
        Error_lists = []
        NumberFile = len(Observed_file)
        InitialData = []
        for i in range(NumberFile):
            print("This is", i, "file")
            Errors = []
            Observed_ = Observed_file[i]
            Simu_ = Simu_file[i]
            ObservedData = PlyData.read(Observed_dir + Observed_)
            ObservedData = ObservedData.elements[0].data
            if i == 0:
                InitialData = ObservedData
            SimuData = PlyData.read(Simu_dir + Simu_)
            SimuData = SimuData.elements[0].data
            SimuData = SimuData[:len(ObservedData)]
            for j in range(len(ObservedData)):
                Errors.append(ComputeDistance(ObservedData[j], SimuData[j]))
            Error_lists.append(Errors)
        NumpyErrors = np.array(Error_lists)
        return NumpyErrors, InitialData

def ComputeDistance(p1, p2):
    error = 0
    for z in range(3):
        error += abs(p1[z] - p2[z]) #absolute value or L2-norm
    return error

def GeneLineChart(LineChartPath, NumpyErrors):
    ErrorTime = np.mean(NumpyErrors, axis=1)
    np.savetxt(LineChartPath,ErrorTime)

def GeneBarChart(BarChartPath, NumpyErrors):
    MeanError = np.mean(NumpyErrors)
    VarError = np.var(NumpyErrors)
    tmp = np.array([MeanError,VarError])
    np.savetxt(BarChartPath,tmp)

def GeneHeatmap(HeatmapPath, NumpyErrors):
    ErrorParticle = np.mean(NumpyErrors, axis=0)
    np.savetxt(HeatmapPath,ErrorParticle)
    return ErrorParticle

if __name__ == "__main__":
    Observed_file = sys.argv[1]
    NoRegiSimu_file = sys.argv[2]
    RegiSimu_file = sys.argv[3]
    NoRegLineChartPath = "./NoRegLineChart.txt"
    NoRegBarChartPath = "./NoRegBarChart.txt"
    NoRegHeatmapPath = "./NoRegHeatmap.txt"
    NumpyErrors, InitialData = ComputeError(Observed_file, NoRegiSimu_file)
    GeneLineChart(NoRegLineChartPath, NumpyErrors)
    GeneBarChart(NoRegBarChartPath, NumpyErrors)
    ErrorParticle = GeneHeatmap(NoRegHeatmapPath, NumpyErrors)

    # RegLineChartPath = "./RegLineChart.txt"
    # RegBarChartPath = "./RegBarChart.txt"
    # RegHeatmapPath = "./RegHeatmap.txt"
    # NumpyErrors, InitialData = ComputeError(Observed_file, RegiSimu_file)
    # GeneLineChart(RegLineChartPath, NumpyErrors)
    # GeneBarChart(RegBarChartPath, NumpyErrors)
    # ErrorParticle = GeneHeatmap(RegHeatmapPath, NumpyErrors)
