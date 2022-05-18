clear all 
close all

syms x1;
syms x2;
syms x3;

A =[0,1,0;
    0,0,1;
    0,0,0];

B = [0;0;1];

p = [-2,-1.5,-2.5];

place(A,B,p)