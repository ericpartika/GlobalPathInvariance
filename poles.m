clear all 
close all

syms x1;
syms x2;
syms x3;

A =[0,1,0;
    0,0,1;
    0,0,0];

B = [0;0;1];

p = [-.1,-.2,-0.3];

place(A,B,p)