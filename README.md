# Triangles

Program checkes whether input triangles are intersecting

## Run the program

```bash
git clone https://github.com/a-palonskaa/Triangles
cd Triangles
```

### To build
```bash
cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build
```

### To run program
```bash
./build/find
```

### To run tests
```bash
./build/run_tests
```

## Input & Output format

input:
```
<n tringles> <tr1_v1 x> <rtr1_v1 y> <tr1_v1 z> <tr1_v2 x> <tr1_v2 y> <tr1_v2 z> <tr1_v3 x> <tr1_v3 y> <tr1_v3 z> ... <>
```

output:
numbers of triangles that has at least one intersection with other in ascending order
```
<tr_a1>
<tr_a2>
<tr_a3>
...
<tr_ak>
```



