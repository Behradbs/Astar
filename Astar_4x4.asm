.data
# 4x4 grid for A* pathfinding example
# Symbols:
#  S = start, G = goal, # = wall, . = open space
#  X = path
grid:
    .byte 'S', '.', '.', 'G'
    .byte '.', '#', '#', '.'
    .byte '.', '#', '.', '.'
    .byte '.', '.', '.', '.'
.align 2

# grid dimensions
width:      .word 4
height:     .word 4

# arrays for search (max 16 cells)
open_x:    .space 16
open_y:    .space 16
open_g:    .space 64
open_f:    .space 64
open_size: .word 0

closed:     .space 16
came_from:  .space 64
g_score:    .space 64

start_idx:  .word 0
goal_idx:   .word 0

# Colors
grass:       .word 0x00FF00  # green
wall_color:  .word 0xFF0000  # red
start_color: .word 0xFFFFFF  # white
goal_color:  .word 0xFFFF00  # yellow
path_color:  .word 0x000000  # black

bitmap_base: .word 0x10008000

# Neighbor offsets: up, down, left, right
# (row, col) deltas
neighbor_deltas:
    .byte -1,0, 1,0, 0,-1, 0,1

.text
.globl main
main:
    # locate start and goal
    jal find_start_goal
    # run A*
    jal astar_search
    # draw grid with path
    jal draw_grid
    li $v0, 10
    syscall

############################################################
# find_start_goal
############################################################
find_start_goal:
    la $t0, grid
    li $t1, 0
find_loop:
    lb $t2, 0($t0)
    li $t3, 'S'
    beq $t2, $t3, store_start
    li $t3, 'G'
    beq $t2, $t3, store_goal
cont_find:
    addi $t0, $t0, 1
    addi $t1, $t1, 1
    li $t4, 16
    blt $t1, $t4, find_loop
    jr $ra
store_start:
    sw $t1, start_idx
    j cont_find
store_goal:
    sw $t1, goal_idx
    j cont_find

############################################################
# Manhattan distance heuristic
############################################################
heuristic:
    lw $t0, goal_idx
    lw $t5, width
    divu $t0, $t5
    mflo $t1      # goal row
    mfhi $t2      # goal col
    sub $t3, $a0, $t1
    abs $t3, $t3
    sub $t4, $a1, $t2
    abs $t4, $t4
    add $v0, $t3, $t4
    jr $ra

############################################################
# push_open
############################################################
push_open:
    lw $t0, open_size
    la $t1, open_x
    la $t2, open_y
    la $t3, open_g
    la $t4, open_f
    add $t1, $t1, $t0
    add $t2, $t2, $t0
    sll $t5, $t0, 2
    add $t3, $t3, $t5
    add $t4, $t4, $t5
    sb $a0, 0($t1)
    sb $a1, 0($t2)
    sw $a2, 0($t3)
    sw $a3, 0($t4)
    addi $t0, $t0, 1
    sw $t0, open_size
    jr $ra

############################################################
# pop_open - returns cell with lowest f
############################################################
pop_open:
    lw $t0, open_size
    beqz $t0, pop_fail
    addi $t0, $t0, -1      # last index
    la $t1, open_f
    lw $t2, 0($t1)
    li $t3, 0
    li $t4, 0
find_min:
    beq $t4, $t0, end_min
    addi $t4, $t4, 1
    addi $t1, $t1, 4
    lw $t5, 0($t1)
    slt $t6, $t5, $t2
    beqz $t6, find_min
    move $t2, $t5
    move $t3, $t4
    j find_min
end_min:
    la $t1, open_x
    la $t2, open_y
    la $t5, open_g
    la $t6, open_f
    add $t1, $t1, $t3
    add $t2, $t2, $t3
    sll $t7, $t3, 2
    add $t5, $t5, $t7
    add $t6, $t6, $t7
    lb $v0, 0($t1)
    lb $v1, 0($t2)
    lw $t8, 0($t5)
    move $v2, $t8

    la $s0, open_x
    add $s0, $s0, $t3
    la $s1, open_y
    add $s1, $s1, $t3
    la $s2, open_g
    add $s2, $s2, $t7
    la $s3, open_f
    add $s3, $s3, $t7
shift_loop:
    bgt $t3, $t0, end_shift
    lb $t9, 1($s0)
    sb $t9, 0($s0)
    lb $t9, 1($s1)
    sb $t9, 0($s1)
    lw $t9, 4($s2)
    sw $t9, 0($s2)
    lw $t9, 4($s3)
    sw $t9, 0($s3)
    addi $s0, $s0, 1
    addi $s1, $s1, 1
    addi $s2, $s2, 4
    addi $s3, $s3, 4
    addi $t3, $t3, 1
    j shift_loop
end_shift:
    lw $t2, open_size
    addi $t2, $t2, -1
    sw $t2, open_size
    jr $ra
pop_fail:
    li $v0, -1
    li $v1, -1
    li $v2, 0
    jr $ra

############################################################
# astar_search
############################################################
astar_search:
    lw $t0, start_idx
    lw $t7, width
    divu $t0, $t7
    mflo $a0     # start row
    mfhi $a1     # start col
    li $a2, 0    # g = 0
    jal heuristic
    move $a3, $v0
    jal push_open
    la $t0, g_score
    sw $zero, 0($t0)
search_loop:
    jal pop_open
    bltz $v0, fail
    move $t0, $v0
    move $t1, $v1
    move $t2, $v2
    lw $t7, width
    mul $t3, $t0, $t7
    add $t3, $t3, $t1
    lw $t4, goal_idx
    beq $t3, $t4, reconstruct
    la $t5, closed
    add $t5, $t5, $t3
    li $t6, 1
    sb $t6, 0($t5)
    li $t6, 0
neighbor:
    la $t7, neighbor_deltas
    add $t7, $t7, $t6
    lb $t8, 0($t7)
    lb $t9, 1($t7)
    add $a0, $t0, $t8
    add $a1, $t1, $t9
    bltz $a0, cont
    bltz $a1, cont
    lw $s0, height
    addi $s0, $s0, -1
    bgt $a0, $s0, cont
    lw $s0, width
    addi $s0, $s0, -1
    bgt $a1, $s0, cont
    lw $s1, width
    mul $s1, $a0, $s1
    add $s1, $s1, $a1
    la $s2, closed
    add $s2, $s2, $s1
    lb $s3, 0($s2)
    bnez $s3, cont
    la $s4, grid
    add $s4, $s4, $s1
    lb $s5, 0($s4)
    li $s6, '#'
    beq $s5, $s6, cont
    addi $s7, $t2, 1
    la $t7, g_score
    sll $s2, $s1, 2
    add $t7, $t7, $s2
    lw $t8, 0($t7)
    beqz $t8, better
    slt $t9, $s7, $t8
    beqz $t9, cont
better:
    sw $s7, 0($t7)
    la $t7, came_from
    add $t7, $t7, $s2
    sw $t3, 0($t7)
    move $a2, $s7
    jal heuristic
    add $a3, $s7, $v0
    jal push_open
cont:
    addi $t6, $t6, 2
    li $s0, 8
    slt $s1, $t6, $s0
    bnez $s1, neighbor
    j search_loop
fail:
    jr $ra

############################################################
# Path reconstruction
############################################################
reconstruct:
    move $t0, $t3
recon_loop:
    lw $t1, start_idx
    beq $t0, $t1, exit
    la $t2, grid
    add $t2, $t2, $t0
    li $t4, 'X'
    sb $t4, 0($t2)
    la $t5, came_from
    sll $t6, $t0, 2
    add $t5, $t5, $t6
    lw $t0, 0($t5)
    j recon_loop
exit:
    jr $ra

############################################################
# draw_grid
############################################################
draw_grid:
    la   $a0, grid           # grid array
    lw   $s4, width
    lw   $s5, height
    li   $s6, 16             # pixel block size

    la   $t1, bitmap_base
    lw   $t1, 0($t1)         # base address of display

    li   $t2, 0              # row index
row_loop:
    li   $t3, 0              # col index
col_loop:
    # Calculate flat index: (row * width) + col
    mul  $t8, $t2, $s4
    add  $t8, $t8, $t3
    add  $t8, $a0, $t8
    lb   $t4, 0($t8)

    # Load default color: grass
    la   $t5, grass
    lw   $t6, 0($t5)

    li   $t7, '#'
    beq  $t4, $t7, is_wall
    li   $t7, 'S'
    beq  $t4, $t7, is_start
    li   $t7, 'G'
    beq  $t4, $t7, is_goal
    li   $t7, 'X'
    beq  $t4, $t7, is_path
    j     draw_block

is_wall:
    la   $t5, wall_color
    lw   $t6, 0($t5)
    j     draw_block
is_start:
    la   $t5, start_color
    lw   $t6, 0($t5)
    j     draw_block
is_goal:
    la   $t5, goal_color
    lw   $t6, 0($t5)
    j     draw_block
is_path:
    la   $t5, path_color
    lw   $t6, 0($t5)

# draw block of pixels for current cell
# (16x16 size)
draw_block:
    move $s0, $t2    # row (grid y)
    move $s1, $t3    # col (grid x)

    li   $t9, 0      # dy = 0
draw_row_pixels:
    li   $s2, 0      # dx = 0
draw_col_pixels:
    # pixel_y = row * size + dy
    # pixel_x = col * size + dx
    mul  $a1, $s0, $s6
    add  $a1, $a1, $t9    # pixel_y

    mul  $a2, $s1, $s6
    add  $a2, $a2, $s2    # pixel_x

    # offset = ((y * 64) + x) * 4
    mul  $t7, $a1, 64
    add  $t7, $t7, $a2
    mul  $t7, $t7, 4
    add  $t7, $t7, $t1

    sw   $t6, 0($t7)

    addi $s2, $s2, 1
    blt  $s2, $s6, draw_col_pixels

    addi $t9, $t9, 1
    blt  $t9, $s6, draw_row_pixels

    addi $t3, $t3, 1
    blt  $t3, $s4, col_loop

    addi $t2, $t2, 1
    blt  $t2, $s5, row_loop

    jr $ra
