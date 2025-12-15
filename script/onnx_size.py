import onnx

m = onnx.load("../model/LR.onnx")  # 或 AB.onnx
for i in m.graph.input:
    shp = [d.dim_value or d.dim_param for d in i.type.tensor_type.shape.dim]
    print("input", i.name, shp)
for o in m.graph.output:
    shp = [d.dim_value or d.dim_param for d in o.type.tensor_type.shape.dim]
    print("output", o.name, shp)
