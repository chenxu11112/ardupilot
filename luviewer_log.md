# 记录: 2022-5-12 
1. 这里倾转的逻辑是采用转换imu坐标系的方法，这样可以避免大角度导致的奇异问题        
2. guide模式可以实现倾转90度，但是loiter还不行，存在问题还需要排查      


# 记录: 2022-5-19
1. 解决了大角度下降的问题       
    在mode_loiter模式中，有这样一段代码： copter.surface_tracking.update_surface_offset();      
    这是用对地激光检测的，倾转之后就不再是对地了，就会下降
