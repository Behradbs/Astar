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

# grid size constant
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

deltas:
    .byte -1,0, 1,0, 0,-1, 0,1

.text
.globl main
main:
    jal find_start_goal
    jal astar_search
    jal draw_grid
    li $v0, 10
    syscall
######################################################################
# find_start_goal
######################################################################
find_start_goal:
    la $t0, grid
    li $t1, 0
find_loop:
    lb $t4, 0($t0)
    li $t5, 'S'
    beq $t4, $t5, store_start
    li $t5, 'G'
    beq $t4, $t5, store_goal
cont_find:
    addi $t0, $t0, 1
    addi $t1, $t1, 1
    blt $t1, 256, find_loop
    jr $ra
store_start:
    sw $t1, start_idx
    j cont_find
store_goal:
    sw $t1, goal_idx
    j cont_find

######################################################################
# heuristic: Manhattan distance to goal
######################################################################
heuristic:
    lw $t0, goal_idx
    li $t5, 16
    divu $t0, $t5
    mflo $t1
    mfhi $t2
    sub $t3, $a0, $t1
    abs $t3, $t3
    sub $t4, $a1, $t2
    abs $t4, $t4
    add $v0, $t3, $t4
    jr $ra

######################################################################
# push_open
######################################################################
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
######################################################################
# pop_open
######################################################################
pop_open:
    lw $t0, open_size
    beqz $t0, pop_fail
    addi $t0, $t0, -1
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
######################################################################
# astar_search
######################################################################
astar_search:
    lw $t0, start_idx
    li $t7, 16
    divu $t0, $t7
    mflo $a0
    mfhi $a1
    li $a2, 0
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
    mul $t3, $t0, 16
    add $t3, $t3, $t1
    lw $t4, goal_idx
    beq $t3, $t4, reconstruct
    la $t5, closed
    add $t5, $t5, $t3
    li $t6, 1
    sb $t6, 0($t5)

    li $t6, 0
neighbor:
    la $t7, deltas
    add $t7, $t7, $t6
    lb $t8, 0($t7)
    lb $t9, 1($t7)
    add $a0, $t0, $t8
    add $a1, $t1, $t9
    bltz $a0, cont
    bltz $a1, cont
    li $s0, 15
    bgt $a0, $s0, cont
    bgt $a1, $s0, cont
    mul $s1, $a0, 16
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
    add $t7, $t7, $s1
    lw $t8, 0($t7)
    beqz $t8, better
    slt $t9, $s7, $t8
    beqz $t9, cont
better:
    sw $s7, 0($t7)
    la $t7, came_from
    add $t7, $t7, $s1
    sw $t3, 0($t7)
    move $a2, $s7
    jal heuristic
    add $a3, $s7, $v0
    jal push_open
cont:
    addi $t6, $t6, 2
    slti $t10, $t6, 8
    bnez $t10, neighbor
    j search_loop
fail:
    jr $ra
######################################################################
# Path reconstruction
######################################################################
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
    add $t5, $t5, $t0
    lw $t0, 0($t5)
    j recon_loop
exit:
    jr $ra

######################################################################
# draw_grid
######################################################################
draw_grid:
    la $a0, grid
    la $t1, bitmap_base
    lw $t1, 0($t1)
    li $t2, 0
    li $s6, 16
r_loop:
    li $t3, 0
c_loop:
    mul $t8, $t2, 16
    add $t8, $t8, $t3
    add $t8, $a0, $t8
    lb $t4, 0($t8)
    la $t5, grass
    lw $t6, 0($t5)
    li $t7, '#'
    beq $t4, $t7, wall
    li $t7, 'S'
    beq $t4, $t7, start
    li $t7, 'G'
    beq $t4, $t7, goal
    li $t7, 'B'
    beq $t4, $t7, bush
    li $t7, 'T'
    beq $t4, $t7, tree
    li $t7, 'P'
    beq $t4, $t7, pond
    li $t7, 'F'
    beq $t4, $t7, flower
    li $t7, 'R'
    beq $t4, $t7, rock
    li $t7, 'C'
    beq $t4, $t7, bench
    li $t7, 'X'
    beq $t4, $t7, path
pixel_draw:
    li $s1, 0
prow:
    li $s0, 0
pcol:
    mul $s2, $t2, 16
    add $s2, $s2, $s1
    mul $s3, $t3, 16
    add $s3, $s3, $s0
    mul $s4, $s2, 256
    add $s4, $s4, $s3
    mul $s4, $s4, 4
    add $s4, $s4, $t1
    sw $t6, 0($s4)
    addi $s0, $s0, 1
    blt $s0, $s6, pcol
    addi $s1, $s1, 1
    blt $s1, $s6, prow
    addi $t3, $t3, 1
    blt $t3, $s6, c_loop
    addi $t2, $t2, 1
    blt $t2, $s6, r_loop
    jr $ra
wall:
    la $t6, wall_color
    lw $t6, 0($t6)
    j pixel_draw
start:
    la $t6, start_color
    lw $t6, 0($t6)
    j pixel_draw
goal:
    la $t6, goal_color
    lw $t6, 0($t6)
    j pixel_draw
bush:
    la $t6, bush_color
    lw $t6, 0($t6)
    j pixel_draw
tree:
    la $t6, tree_color
    lw $t6, 0($t6)
    j pixel_draw
pond:
    la $t6, pond_color
    lw $t6, 0($t6)
    j pixel_draw
flower:
    la $t6, flower_color
    lw $t6, 0($t6)
    j pixel_draw
rock:
    la $t6, rock_color
    lw $t6, 0($t6)
    j pixel_draw
bench:
    la $t6, bench_color
    lw $t6, 0($t6)
    j pixel_draw
path:
    la $t6, path_color
    lw $t6, 0($t6)
    j pixel_draw
