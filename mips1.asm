        .data
# Grid definition (4×4)

grid:
        .byte 'S', '.', '.', 'G'
        .byte '.', '#', '#', '.'
        .byte '.', '#', '.', '.'
        .byte '.', '.', '.', '.'
.align 2


# Color constants

grass:       .word 0x00FF00  # green
wall_color:  .word 0xFF0000  # red
start_color: .word 0xFFFFFF  # white
goal_color:  .word 0xFFFF00  # yellow

bitmap_base: .word 0x10008000

# Message for console

msg:    .asciiz "Shortest path = "
newline:.asciiz "\n"

        .text
        .globl main


main:
    
    # PART 1: Compute Manhattan #
    
    # Find 'S' index
    
    la   $t0, grid
    li   $t1, 0
findS:

    lb   $t2, 0($t0)
    li   $t3, 'S'
    beq  $t2, $t3, gotS
    addi $t0, $t0, 1
    addi $t1, $t1, 1
    blt  $t1, 16, findS

gotS:

    # row_S = t1/4, col_S = t1%4
    
    li   $t4, 4
    div  $t1, $t4
    mflo $s0      # s0=row_S
    mfhi $s1      # s1=col_S

    # Find 'G' index
    
    la   $t0, grid
    li   $t1, 0
findG:

    lb   $t2, 0($t0)
    li   $t3, 'G'
    beq  $t2, $t3, gotG
    addi $t0, $t0, 1
    addi $t1, $t1, 1
    blt  $t1, 16, findG

gotG:

    # row_G = t1/4, col_G = t1%4
    
    li   $t4, 4
    div  $t1, $t4
    mflo $s2      # s2=row_G
    mfhi $s3      # s3=col_G

    # Compute abs(row_S - row_G)
    
    sub  $t5, $s0, $s2
    bltz $t5, absR
    move $t6, $t5
    j    gotR
    
absR:

    neg  $t6, $t5
    
gotR:

    # Compute abs(col_S - col_G)
    
    sub  $t7, $s1, $s3
    bltz $t7, absC
    move $t8, $t7
    j    gotC
    
absC:

    neg  $t8, $t7
    
gotC:

    add  $t9, $t6, $t8      # t9 = Manhattan distance


    # Print "Shortest path = "
    
    li   $v0, 4
    la   $a0, msg
    syscall
    
    # Print the number
    
    li   $v0, 1
    move $a0, $t9
    syscall
    
    # Print newline
    
    li   $v0, 4
    la   $a0, newline
    syscall

  
    # PART 2: Draw the grid in bitmap #
    
    la   $a0, grid           # base of grid[]
    li   $s4, 4              # width
    li   $s5, 4              # height
    li   $s6, 16             # block size

    la   $t1, bitmap_base
    lw   $t1, 0($t1)         # display base

    li   $t2, 0              # row
    
row_loop:

    li   $t3, 0              # col
    
col_loop:

    # index = row*width + col
    
    mul  $t8, $t2, $s4
    add  $t8, $t8, $t3
    add  $t8, $a0, $t8
    lb   $t4, 0($t8)

    # default grass
    
    la   $t5, grass
    lw   $t6, 0($t5)

    # override for wall/start/goal
    
    li   $t7, '#'
    beq  $t4, $t7, is_wall
    li   $t7, 'S'
    beq  $t4, $t7, is_start
    li   $t7, 'G'
    beq  $t4, $t7, is_goal
    j    draw_block

is_wall:

    la   $t5, wall_color
    lw   $t6, 0($t5)
    j    draw_block
is_start:

    la   $t5, start_color
    lw   $t6, 0($t5)
    j    draw_block
is_goal:

    la   $t5, goal_color
    lw   $t6, 0($t5)

draw_block:

    move $s0, $t2
    move $s1, $t3
    li   $t9, 0
    
draw_row:

    li   $s2, 0
    
draw_col:

    mul  $a1, $s0, $s6
    add  $a1, $a1, $t9
    mul  $a2, $s1, $s6
    add  $a2, $a2, $s2
    mul  $t7, $a1, 64
    add  $t7, $t7, $a2
    mul  $t7, $t7, 4
    add  $t7, $t7, $t1
    sw   $t6, 0($t7)
    addi $s2, $s2, 1
    blt  $s2, $s6, draw_col
    addi $t9, $t9, 1
    blt  $t9, $s6, draw_row

    addi $t3, $t3, 1
    blt  $t3, $s4, col_loop
    addi $t2, $t2, 1
    blt  $t2, $s5, row_loop

    # exit
    
    li   $v0, 10
    syscall
