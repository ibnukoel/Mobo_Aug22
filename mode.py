mode = 5
data_terminal = 0
#mode 0 = ambil data dengan keybo2rd
#mode 1 = run
#mode 2 = run dengan koreksi posisi
#mode 3 = mode2 wit output deltarho
#mode 4 = test kecepatan putar
simpanData = 0
#1 = simpan data
#0 = tidak simpan

#Adjustable Parameter
d_rho1 = 15
d_rho2 = 2
v = 0.1 #kecepatan lurus
v_r_asli = 0.5  #kecepatan saat putar
#kecepatan Robot ketika koreksi
v_k = 3
#testing
v_r_test = [2,2.5,3]
#v_r_test = [0.5,0.6,0.7,0.8,0.9,1,0.9,1,1.5,2,2.5,3]
sudutRobot_test = [-90,90,180,0]





init = [0, 0]
goal = [3,5]