.data
# 16×16 “garden” map (each cell = 16×16 pixels on a 256×256 bitmap)
# Symbols:
#   S = start, G = goal, # = wall, B = bush, T = tree
#   P = pond, F = flower, R = rock, C = bench, . = grass
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

# Color definitions
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

bitmap_base:  .word 0x10008000

.text
.globl main
main:
    la   $a0, grid         # $a0 = address of grid[]
    li   $t1, 0x10008000   # $t1 = bitmap display base
    li   $t2, 0            # row index = 0
    li   $s6, 16           # grid size & block size

row_loop:
    li   $t3, 0            # col index = 0

col_loop:
    # compute symbol address: addr = grid + (row*16 + col)
    mul  $t8, $t2, 16
    add  $t8, $t8, $t3
    add  $t8, $a0, $t8
    lb   $t4, 0($t8)       # $t4 = grid[row][col]

    # default color = grass
    la   $t5, grass
    lw   $t6, 0($t5)

    # override on special symbols
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

draw_pixel:
    li   $s1, 0            # block‐row offset
draw_block_row:
    li   $s0, 0            # block‐col offset
draw_block_col:
    # compute display coords
    mul  $s2, $t2, 16
    add  $s2, $s2, $s1     # y = row*16 + s1
    mul  $s3, $t3, 16
    add  $s3, $s3, $s0     # x = col*16 + s0

    # compute address = ((y*256 + x)*4) + base
    mul  $s4, $s2, 256
    add  $s4, $s4, $s3
    mul  $s4, $s4, 4
    add  $s4, $s4, $t1

    sw   $t6, 0($s4)       # write pixel color

    addi $s0, $s0, 1
    blt  $s0, $s6, draw_block_col

    addi $s1, $s1, 1
    blt  $s1, $s6, draw_block_row

    # next column
    addi $t3, $t3, 1
    blt  $t3, $s6, col_loop

    # next row
    addi $t2, $t2, 1
    blt  $t2, $s6, row_loop

    li   $v0, 10
    syscall

# -- color‐override labels --
is_wall:
    la   $t5, wall_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_start:
    la   $t5, start_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_goal:
    la   $t5, goal_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_bush:
    la   $t5, bush_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_tree:
    la   $t5, tree_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_pond:
    la   $t5, pond_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_flower:
    la   $t5, flower_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_rock:
    la   $t5, rock_color
    lw   $t6, 0($t5)
    j    draw_pixel

is_bench:
    la   $t5, bench_color
    lw   $t6, 0($t5)
    j    draw_pixel
