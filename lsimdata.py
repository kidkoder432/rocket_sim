from landing_sim import *
import plotly.graph_objs as go

data = []
ms = []

for m in range(500, 700, 10):
    ms.append(m)
    data.append([])
    ts = []
    if m % 5 == 0: print(m, end=" ")
    for t in range(300, 700, 5):
        ts.append(t)
        if t % 5 == 0: print(t)
        mx = m / 1000
        tx = t / 100

        data[-1].append(-sim(tx, mx))

    print()

print(len(data), len(data[0]), len(ms), len(ts))
fig = go.Figure(go.Heatmap(x=ts, y=ms, z=data))


def onclick(trace, points, selector):
    print(points)


fig.data[0].on_click(onclick)

fig
