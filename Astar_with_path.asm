.data
# 16x16 garden map used for pathfinding and display
# Symbols:
#   S = start, G = goal, # = wall, B = bush, T = tree
#   P = pond, F = flower, R = rock, C = bench, . = grass
#   X = path
grid:
    .byte 'S','.','.','.','B','.','.','.','.','.','.','.','.','.','.','G'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','P','P','P','P','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','P','.','.','P','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','P','.','.','P','.','.','.','F','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','B','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','T','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','F','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','R','R','.','.','.','.','.','.','.'
    .byte '.','.','C','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'
    .byte '.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','G'

grid_size:      .word 16

# arrays for A* search (max 256 cells)
open_x:     .space 256
open_y:     .space 256
open_g:     .space 1024    # word per cell
open_f:     .space 1024
open_size:  .word 0

closed:     .space 256     # byte flags
came_from:  .space 1024    # parent index per cell
g_score:    .space 1024

start_idx:  .word 0
goal_idx:   .word 0

# color definitions
grass:        .word 0x00FF00
bush_color:   .word 0x008000
tree_color:   .word 0x8B4513
pond_color:   .word 0x0000FF
flower_color: .word 0xFF00FF
rock_color:   .word 0x808080
bench_color:  .word 0xA52A2A
wall_color:   .word 0xFF0000
start_color:  .word 0xFFFFFF
goal_color:   .word 0xFFFF00
path_color:   .word 0x000000

bitmap_base:  .word 0x10008000

.text
.globl main
main:
    # find start and goal indices
    jal  find_start_goal

    # run A* search to overlay path using 'X'
    jal  astar_search

    # draw resulting grid to display
    jal  draw_grid

    li   $v0, 10
    syscall

######################################################################
# Routine: find_start_goal
# Scans the grid to locate 'S' and last 'G'
# returns indices in start_idx and goal_idx
######################################################################
find_start_goal:
    la   $t0, grid
    li   $t1, 0          # index
    li   $t2, 0          # found start?
    li   $t3, 0          # found goal?
find_loop:
    lb   $t4, 0($t0)
    li   $t5, 'S'
    beq  $t4, $t5, store_start
    li   $t5, 'G'
    beq  $t4, $t5, store_goal
cont_find:
    addi $t0, $t0, 1
    addi $t1, $t1, 1
    blt  $t1, 256, find_loop
    jr   $ra
store_start:
    sw   $t1, start_idx
    move $t2, $t1
    j    cont_find
store_goal:
    sw   $t1, goal_idx
    move $t3, $t1
    j    cont_find

######################################################################
# Routine: heuristic
# Input: $a0 = row, $a1 = col
# Output: $v0 = Manhattan distance to goal
######################################################################
heuristic:
    # row difference
    lw   $t0, goal_idx
    li   $t5, 16
    divu $t0, $t5
    mflo $t1          # goal_row
    mfhi $t2          # goal_col (using remainder after dividing by 16)
    sub  $t3, $a0, $t1
    abs  $t3, $t3
    sub  $t4, $a1, $t2
    abs  $t4, $t4
    add  $v0, $t3, $t4
    jr   $ra

######################################################################
# Routine: push_open
# Inserts node into open set sorted by f
# Input: $a0=row, $a1=col, $a2=g, $a3=f
######################################################################
push_open:
    lw   $t0, open_size            # current size
    la   $t1, open_x
    la   $t2, open_y
    la   $t3, open_g
    la   $t4, open_f
    add  $t1, $t1, $t0             # byte offset = index
    add  $t2, $t2, $t0
    sll  $t5, $t0, 2               # word offset = index*4
    add  $t3, $t3, $t5
    add  $t4, $t4, $t5
    sb   $a0, 0($t1)
    sb   $a1, 0($t2)
    sw   $a2, 0($t3)
    sw   $a3, 0($t4)

    addi $t0, $t0, 1
    sw   $t0, open_size
    jr   $ra

######################################################################
# Routine: pop_open
# Removes node with lowest f from open set
# Output: $v0=row, $v1=col, $v2=g
######################################################################
pop_open:
    lw   $t0, open_size         # number of items
    beqz $t0, pop_fail
    addi $t0, $t0, -1           # last index
    la   $t1, open_f            # pointer to f array

    # find index of minimum f
    lw   $t2, 0($t1)            # current min f
    li   $t3, 0                 # min index
    li   $t4, 0                 # loop index
find_min_loop:
    beq  $t4, $t0, end_find_min
    addi $t4, $t4, 1
    addi $t1, $t1, 4
    lw   $t5, 0($t1)
    slt  $t6, $t5, $t2
    beqz $t6, find_min_loop
    move $t2, $t5
    move $t3, $t4
    j    find_min_loop
end_find_min:

    # load node at index t3
    la   $t1, open_x
    la   $t2, open_y
    la   $t5, open_g
    la   $t6, open_f
    add  $t1, $t1, $t3
    add  $t2, $t2, $t3
    sll  $t7, $t3, 2
    add  $t5, $t5, $t7
    add  $t6, $t6, $t7
    lb   $v0, 0($t1)
    lb   $v1, 0($t2)
    lw   $v2, 0($t5)

    # shift elements left to fill gap
    la   $t8, open_x
    add  $t8, $t8, $t3
    la   $t9, open_y
    add  $t9, $t9, $t3
    la   $s0, open_g
    add  $s0, $s0, $t7
    la   $s1, open_f
    add  $s1, $s1, $t7
shift_loop:
    addi $t3, $t3, 1
    bgt  $t3, $t0, end_shift
    # copy from index t3 to previous slot
    la   $s2, open_x
    add  $s2, $s2, $t3
    lb   $s3, 0($s2)
    sb   $s3, 0($t8)
    la   $s2, open_y
    add  $s2, $s2, $t3
    lb   $s3, 0($s2)
    sb   $s3, 0($t9)
    la   $s2, open_g
    sll  $s4, $t3, 2
    add  $s2, $s2, $s4
    lw   $s3, 0($s2)
    sw   $s3, 0($s0)
    la   $s2, open_f
    add  $s2, $s2, $s4
    lw   $s3, 0($s2)
    sw   $s3, 0($s1)
    addi $t8, $t8, 1
    addi $t9, $t9, 1
    addi $s0, $s0, 4
    addi $s1, $s1, 4
    j    shift_loop
end_shift:
    lw   $t2, open_size
    addi $t2, $t2, -1
    sw   $t2, open_size
    jr   $ra
pop_fail:
    li   $v0, -1
    li   $v1, -1
    li   $v2, 0
    jr   $ra

######################################################################
# Routine: astar_search
# Runs A* algorithm to mark path with 'X'
######################################################################
astar_search:
    # initialize
    lw   $t0, start_idx
    li   $t7, 16
    divu $t0, $t7
    mflo $a0         # row
    mfhi $a1         # col
    li   $a2, 0      # g = 0
    jal  heuristic
    move $a3, $v0    # f = heuristic(start)
    jal  push_open
    la   $t0, g_score
    sw   $zero, 0($t0)

search_loop:
    jal  pop_open
    bltz $v0, search_fail
    move $t0, $v0    # row
    move $t1, $v1    # col
    move $t2, $v2    # g
    # compute idx = row*16 + col
    mul  $t3, $t0, 16
    add  $t3, $t3, $t1
    # check if goal
    lw   $t4, goal_idx
    beq  $t3, $t4, reconstruct
    # mark closed
    la   $t5, closed
    add  $t5, $t5, $t3
    li   $t6, 1
    sb   $t6, 0($t5)

    # explore 4 neighbors
    li   $t6, 0
neighbor_loop:
    la   $t7, deltas
    add  $t7, $t7, $t6
    lb   $t8, 0($t7)
    lb   $t9, 1($t7)
    add  $a0, $t0, $t8
    add  $a1, $t1, $t9
    bltz $a0, next_neighbor
    bltz $a1, next_neighbor
    li   $s0, 15
    bgt  $a0, $s0, next_neighbor
    bgt  $a1, $s0, next_neighbor
    mul  $s1, $a0, 16
    add  $s1, $s1, $a1
    la   $s2, closed
    add  $s2, $s2, $s1
    lb   $s3, 0($s2)
    bnez $s3, next_neighbor
    la   $s4, grid
    add  $s4, $s4, $s1
    lb   $s5, 0($s4)
    li   $s6, '#'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'B'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'T'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'P'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'F'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'R'
    beq  $s5, $s6, next_neighbor
    li   $s6, 'C'
    beq  $s5, $s6, next_neighbor
    # tentative_g = g + 1
    addi $s7, $t2, 1
    la   $t7, g_score
    add  $t7, $t7, $s1
    lw   $t8, 0($t7)
    beqz $t8, better_path
    slt  $t9, $s7, $t8
    beqz $t9, next_neighbor
better_path:
    sw   $s7, 0($t7)
    la   $t7, came_from
    add  $t7, $t7, $s1
    sw   $t3, 0($t7)
    move $a0, $a0      # neighbor row already in $a0
    move $a1, $a1      # neighbor col already in $a1
    move $a2, $s7      # g cost
    jal  heuristic
    add  $a3, $s7, $v0
    jal  push_open
next_neighbor:
    addi $t6, $t6, 2
    slti $t7, $t6, 8
    bnez $t7, neighbor_loop
    j    search_loop
search_fail:
    jr   $ra

######################################################################
# Path reconstruction
######################################################################
reconstruct:
    move $t0, $t3  # current index = goal index
reconstruct_loop:
    lw   $t1, start_idx
    beq  $t0, $t1, draw_exit
    la   $t2, grid
    add  $t3, $t2, $t0
    li   $t4, 'X'
    sb   $t4, 0($t3)
    la   $t5, came_from
    add  $t5, $t5, $t0
    lw   $t0, 0($t5)
    j    reconstruct_loop

draw_exit:
    jr   $ra

######################################################################
# Deltas for neighbor offsets
######################################################################
deltas:
    .byte -1,0, 1,0, 0,-1, 0,1

######################################################################
# Routine: draw_grid
# Displays grid to bitmap memory using color table
######################################################################
draw_grid:
    la   $a0, grid         # $a0 = address of grid[]
    li   $t1, 0x10008000   # $t1 = bitmap display base
    li   $t2, 0            # row index = 0
    li   $s6, 16           # grid size & block size

row_loop:
    li   $t3, 0            # col index = 0

col_loop:
    mul  $t8, $t2, 16
    add  $t8, $t8, $t3
    add  $t8, $a0, $t8
    lb   $t4, 0($t8)

    la   $t5, grass
    lw   $t6, 0($t5)

    li   $t7, '#'
    beq  $t4, $t7, is_wall
    li   $t7, 'S'
    beq  $t4, $t7, is_start
    li   $t7, 'G'
    beq  $t4, $t7, is_goal
    li   $t7, 'B'
    beq  $t4, $t7, is_bush
    li   $t7, 'T'
    beq  $t4, $t7, is_tree
    li   $t7, 'P'
    beq  $t4, $t7, is_pond
    li   $t7, 'F'
    beq  $t4, $t7, is_flower
    li   $t7, 'R'
    beq  $t4, $t7, is_rock
    li   $t7, 'C'
    beq  $t4, $t7, is_bench
    li   $t7, 'X'
    beq  $t4, $t7, is_path

write_pixel:
    li   $s1, 0
pixel_row_loop:
    li   $s0, 0
pixel_col_loop:
    mul  $s2, $t2, 16
    add  $s2, $s2, $s1
    mul  $s3, $t3, 16
    add  $s3, $s3, $s0
    mul  $s4, $s2, 256
    add  $s4, $s4, $s3
    mul  $s4, $s4, 4
    add  $s4, $s4, $t1
    sw   $t6, 0($s4)
    addi $s0, $s0, 1
    blt  $s0, $s6, pixel_col_loop
    addi $s1, $s1, 1
    blt  $s1, $s6, pixel_row_loop

    addi $t3, $t3, 1
    blt  $t3, $s6, col_loop

    addi $t2, $t2, 1
    blt  $t2, $s6, row_loop

    jr   $ra

is_wall:
    la   $t5, wall_color
    lw   $t6, 0($t5)
    j    write_pixel
is_start:
    la   $t5, start_color
    lw   $t6, 0($t5)
    j    write_pixel
is_goal:
    la   $t5, goal_color
    lw   $t6, 0($t5)
    j    write_pixel
is_bush:
    la   $t5, bush_color
    lw   $t6, 0($t5)
    j    write_pixel
is_tree:
    la   $t5, tree_color
    lw   $t6, 0($t5)
    j    write_pixel
is_pond:
    la   $t5, pond_color
    lw   $t6, 0($t5)
    j    write_pixel
is_flower:
    la   $t5, flower_color
    lw   $t6, 0($t5)
    j    write_pixel
is_rock:
    la   $t5, rock_color
    lw   $t6, 0($t5)
    j    write_pixel
is_bench:
    la   $t5, bench_color
    lw   $t6, 0($t5)
    j    write_pixel
is_path:
    la   $t5, path_color
    lw   $t6, 0($t5)
    j    write_pixel
