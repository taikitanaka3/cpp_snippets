: [ RUN ] almost_same_point.spline_linear_cmp

```
TEST(almost_same_point, spline_linear_cmp) {
  // case Too narrow
  std::vector<double> base_keys ={0.0,1.0,1.0001,2.0,3.0,4.0};
  std::vector<double> base_vals ={0.0,0.0,0.1   ,0.1,0.1,0.1};
  std::vector<double> query_keys={0.0,1.0,1.5   ,2.0,3.0,4.0};
}

TEST(nominal, spline_linear_cmp) {
  // case Too narrow
  std::vector<double> base_keys ={0.0,1.0,1.001,2.0,3.0,4.0};
  std::vector<double> base_vals ={0.0,0.0,0.001   ,0.001,0.001,0.001};
  std::vector<double> query_keys={0.0,1.0,1.5   ,2.0,3.0,4.0};
}
```

```test
1: [----------] 1 test from almost_same_point
1: [ RUN      ] almost_same_point.spline_linear_cmp
1: s key : 0 s val : 0
1: l key : 0 l val : 0
1: s key : 1 s val : 0
1: l key : 1 l val : 0
1: s key : 1.5 s val : 137.592
1: l key : 1.5 l val : 0.1
1: s key : 2 s val : 0.1
1: l key : 2 l val : 0.1
1: s key : 3 s val : 0.1
1: l key : 3 l val : 0.1
1: s key : 4 s val : 0.1
1: l key : 4 l val : 0.1
1: case easier
1: s key : 0 s val : 0
1: l key : 0 l val : 0
1: s key : 1 s val : 0
1: l key : 1 l val : 0
1: s key : 1.5 s val : 13.8418
1: l key : 1.5 l val : 0.1
1: s key : 2 s val : 0.1
1: l key : 2 l val : 0.1
1: s key : 3 s val : 0.1
1: l key : 3 l val : 0.1
1: s key : 4 s val : 0.1
1: l key : 4 l val : 0.1
1: [       OK ] almost_same_point.spline_linear_cmp (0 ms)
1: [----------] 1 test from almost_same_point (0 ms total)
1:
1: [----------] 1 test from nominal
1: [ RUN      ] nominal.spline_linear_cmp
1: s key : 0 s val : 0
1: l key : 0 l val : 0
1: s key : 1 s val : 0
1: l key : 1 l val : 0
1: s key : 1.5 s val : 0.138418
1: l key : 1.5 l val : 0.001
1: s key : 2 s val : 0.001
1: l key : 2 l val : 0.001
1: s key : 3 s val : 0.001
1: l key : 3 l val : 0.001
1: s key : 4 s val : 0.001
1: l key : 4 l val : 0.001
1: case easier
1: s key : 0 s val : 0
1: l key : 0 l val : 0
1: s key : 1 s val : 0
1: l key : 1 l val : 0
1: s key : 1.5 s val : 0.00226489
1: l key : 1.5 l val : 0.001
1: s key : 2 s val : 0.001
1: l key : 2 l val : 0.001
1: s key : 3 s val : 0.001
1: l key : 3 l val : 0.001
1: s key : 4 s val : 0.001
1: l key : 4 l val : 0.001
```
