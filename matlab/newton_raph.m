clear;
syms x;
f = x*x - 2;
df = diff(f)
a = 100023

for i = 1:20
    a = a - double(subs(f,a)) / double(subs(df,a));
end
a