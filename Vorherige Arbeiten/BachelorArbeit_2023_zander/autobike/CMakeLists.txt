cmake_minimum_required(VERSION 3.2)
project(autobike)

# target_include_directories(myprojectname PUBLIC /usr/include)

add_executable(autobike 
        autobike.c 
        interfaces/ab_i2c.c 
        interfaces/ab_bno055.c 
        interfaces/ab_pid.c 
        interfaces/ab_md49.c 
        ab_error_handling.c
        interfaces/ab_bts7960.c
        interfaces/ab_hnm.c
        interfaces/ab_io.c
        interfaces/ab_remote.c)
        
        
add_executable(remote_test 
        tests/remote_test.c 
        interfaces/ab_remote.c
        ab_error_handling.c
        interfaces/ab_io.c)

add_executable(hnm_remote_test 
        tests/hnm_remote_test.c 
        interfaces/ab_remote.c
        ab_error_handling.c
        interfaces/ab_io.c
        interfaces/ab_pid.c
        interfaces/ab_hnm.c)      

add_executable(hnm_test 
        tests/hnm_test.c
        ab_error_handling.c
        interfaces/ab_io.c
        interfaces/ab_hnm.c)   

add_executable(steering_test 
        tests/steering_test.c
        ab_error_handling.c
        interfaces/ab_bts7960.c
        interfaces/ab_pid.c
        interfaces/ab_md49.c 
        interfaces/ab_io.c)  

add_executable(steering_pid_test 
        tests/steering_pid_test.c 
        interfaces/ab_pid.c 
        interfaces/ab_md49.c 
        ab_error_handling.c
        interfaces/ab_bts7960.c
        interfaces/ab_io.c)
        
add_executable(hnm_pid_test 
        tests/hnm_pid_test.c 
        interfaces/ab_pid.c 
        ab_error_handling.c
        interfaces/ab_hnm.c     
        interfaces/ab_io.c)

add_executable(md49_polling_test
        tests/md49_polling_test.c
        ab_error_handling.c
        interfaces/ab_md49.c)  

add_executable(bno_test 
        tests/bno_test.c 
        ab_error_handling.c
        interfaces/ab_pid.c
        interfaces/ab_i2c.c 
        interfaces/ab_bno055.c)    
        
add_executable(reset_autobike 
        reset_autobike.c 
        interfaces/ab_i2c.c 
        interfaces/ab_bno055.c 
        interfaces/ab_pid.c 
        interfaces/ab_md49.c 
        ab_error_handling.c
        interfaces/ab_bts7960.c
        interfaces/ab_hnm.c
        interfaces/ab_io.c
        interfaces/ab_remote.c)
        
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(autobike PRIVATE Threads::Threads)

target_link_libraries(steering_pid_test PRIVATE Threads::Threads)
target_link_libraries(hnm_pid_test PRIVATE Threads::Threads)

# target_compile_options(myprojectfile -Wall)
