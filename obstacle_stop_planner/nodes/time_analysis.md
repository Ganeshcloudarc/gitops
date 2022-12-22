
# Details
`rosparam set use_sim_time true`

 ```rosbag play boson_2022-09-27-14-54-09.bag  /local_gps_trajectory:=/local_gps_trajectory1 /collision_points:=cp1 collision_velocity_marker:=cv --clock```
 
 dev-tada machine, 32 Gb NRU.

## with tf:
    ime taken for a loop is {0.10952472686767578}
    time taken for a loop is {0.12091827392578125}
    time taken for a loop is {0.17897629737854004}
    time taken for a loop is {0.12712717056274414}
    time taken for a loop is {0.13098573684692383}
    time taken for a loop is {0.12354111671447754}
    time taken for a loop is {0.18720436096191406}
    time taken for a loop is {0.13651800155639648}
    time taken for a loop is {0.13990235328674316}
    time taken for a loop is {0.14139270782470703}
    time taken for a loop is {0.10241436958312988}
    time taken for a loop is {0.14041590690612793}
    time taken for a loop is {0.1267380714416504}
    time taken for a loop is {0.07471680641174316}
    time taken for a loop is {0.0881812572479248}
    time taken for a loop is {0.07918977737426758}
    time taken for a loop is {0.05248379707336426}
    time taken for a loop is {0.07034850120544434}
    time taken for a loop is {0.07151365280151367}
    time taken for a loop is {0.13018274307250977}
    time taken for a loop is {0.11818671226501465}
    time taken for a loop is {0.12859845161437988}
    time taken for a loop is {0.13561177253723145}
    time taken for a loop is {0.12488818168640137}
    time taken for a loop is {0.06919097900390625}
    time taken for a loop is {0.07579970359802246}
    time taken for a loop is {0.13121342658996582}

## without tf:
    time taken for a loop is {0.05218076705932617}
    time taken for a loop is {0.04850602149963379}
    time taken for a loop is {0.042173147201538086}
    time taken for a loop is {0.027933597564697266}
    time taken for a loop is {0.025025367736816406}
    time taken for a loop is {0.028742313385009766}
    time taken for a loop is {0.03272271156311035}
    time taken for a loop is {0.03422141075134277}
    time taken for a loop is {0.041420936584472656}
    time taken for a loop is {0.04464459419250488}
    time taken for a loop is {0.04261326789855957}
    time taken for a loop is {0.036881446838378906}
    time taken for a loop is {0.031044483184814453}
    time taken for a loop is {0.028872966766357422}
    time taken for a loop is {0.03472328186035156}
    time taken for a loop is {0.02859973907470703}
    time taken for a loop is {0.03827619552612305}
    time taken for a loop is {0.04966235160827637}
    time taken for a loop is {0.03131508827209473}
    time taken for a loop is {0.03985595703125}
    time taken for a loop is {0.029493331909179688}
    time taken for a loop is {0.04627108573913574}
    time taken for a loop is {0.0344691276550293}
    time taken for a loop is {0.04017162322998047}
    time taken for a loop is {0.041013479232788086}
    time taken for a loop is {0.03514552116394043}
    time taken for a loop is {0.03355765342712402}
    time taken for a loop is {0.03478503227233887}
    time taken for a loop is {0.02770519256591797}
    time taken for a loop is {0.0434107780456543}
    time taken for a loop is {0.05295252799987793}
    time taken for a loop is {0.038356781005859375}
    time taken for a loop is {0.02917194366455078}

## when all points search is enabled:
    time taken for a loop is: 0.03834891319274902 , count :61
    time taken for a loop is: 0.03512167930603027 , count :61
    time taken for a loop is: 0.03089737892150879 , count :61
    time taken for a loop is: 0.043360233306884766 , count :61
    time taken for a loop is: 0.04121565818786621 , count :61
    time taken for a loop is: 0.03505086898803711 , count :61
    time taken for a loop is: 0.02964162826538086 , count :61
    time taken for a loop is: 0.032218217849731445 , count :61
    time taken for a loop is: 0.0290372371673584 , count :61
    time taken for a loop is: 0.03144478797912598 , count :61
    time taken for a loop is: 0.034332990646362305 , count :61
    time taken for a loop is: 0.03357100486755371 , count :61
    time taken for a loop is: 0.034010887145996094 , count :61
    time taken for a loop is: 0.029618263244628906 , count :61
    time taken for a loop is: 0.03549933433532715 , count :61
    time taken for a loop is: 0.031354427337646484 , count :61
    time taken for a loop is: 0.030147314071655273 , count :61
    time taken for a loop is: 0.041853904724121094 , count :61
    time taken for a loop is: 0.03363966941833496 , count :61
    time taken for a loop is: 0.03342771530151367 , count :61
    time taken for a loop is: 0.029888153076171875 , count :61
    time taken for a loop is: 0.03157687187194824 , count :61
    time taken for a loop is: 0.036559343338012695 , count :61
    time taken for a loop is: 0.03742027282714844 , count :61
    time taken for a loop is: 0.0385127067565918 , count :61
    time taken for a loop is: 0.03256344795227051 , count :61
    time taken for a loop is: 0.039285898208618164 , count :61
    time taken for a loop is: 0.033360958099365234 , count :61
    time taken for a loop is: 0.03213381767272949 , count :61
    time taken for a loop is: 0.043036699295043945 , count :61
    time taken for a loop is: 0.04017210006713867 , count :61
    time taken for a loop is: 0.03834390640258789 , count :61
    time taken for a loop is: 0.03218269348144531 , count :61
    time taken for a loop is: 0.039257049560546875 , count :61
    time taken for a loop is: 0.03222203254699707 , count :61


## time to search for a point on kdtree
    time to search tree 0.00036978721618652344
    time to search tree 0.00025177001953125
    time to search tree 0.0003001689910888672
    time to search tree 0.00018787384033203125
    time to search tree 0.0003094673156738281
    time to search tree 0.00024080276489257812
    time to search tree 0.000354766845703125


## time to build kd tree 
        0.0006315708160400391
        time to build tree 0.0006284713745117188
        with zed came :- 0.0007026195526123047: 0.00022912025451660156

## when zed_wrapper is running
    time taken for a loop is: 0.034111976623535156 , count :60,

    time taken for a loop is: 0.12072229385375977 , count :69



## no zed camera 
    time taken for a loop is: 0.03484773635864258 , count :67



## when 100 meters are kept as look ahead collison check
time taken for a loop is: 0.4753849506378174 , count :698

## when 20 meters are kept
    time taken for a loop is: 0.07242774963378906 , count :132
    time to build tree 0.0006184577941894531
    time taken for a loop is: 0.0656888484954834 , count :132
    time to build tree 0.0006551742553710938
    time taken for a loop is: 0.07040262222290039 , count :132
    time to build tree 0.0006260871887207031
    time taken for a loop is: 0.07007193565368652 , count :132
    time to build tree 0.0006155967712402344
    time taken for a loop is: 0.06272721290588379 , count :132
    time to build tree 0.0006434917449951172
    time taken for a loop is: 0.06702470779418945 , count :132
    time to build tree 0.000530242919921875

## when 30 meters are kept
    time taken for a loop is: 0.1960444450378418 , count :227

---------

time taken for a loop is: 2.832885980606079 , count :5504


time taken for laser scan conversion is tf: 0.10142731666564941 
time taken for laser scan conversion is no tf: 0.002649545669555664




time taken for laser scan conversion is tf: latest, 0.053977251052856445 

wait tf  0.10399723052978516