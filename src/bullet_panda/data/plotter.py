from bokeh.io import show
from bokeh.plotting import figure


class Plotter:
    def __init__(self, title, axis_labels, plot_bounds, plot_tickers, plot_size=(400, 400)):
        self.fig = figure(match_aspect=True, plot_width=plot_size[0], plot_height=plot_size[1],
                          title=title, x_axis_label=axis_labels[0], y_axis_label=axis_labels[1],
                          x_range=(plot_bounds[0], plot_bounds[1]),
                          y_range=(plot_bounds[2], plot_bounds[3]))

        self.fig.title.text_font_size = "14pt"

        self.fig.xaxis.ticker = plot_tickers[0]
        self.fig.xaxis.major_label_text_font_size = "7pt"
        self.fig.xaxis.axis_label_text_font_size = '12pt'
        self.fig.xaxis.axis_label_text_font_style = 'normal'
        self.fig.xaxis.minor_tick_line_width = 0
        self.fig.xaxis.major_tick_in = 0
        self.fig.xaxis.axis_line_color = '#444444'
        self.fig.xaxis.major_tick_line_color = '#444444'
        self.fig.xaxis.formatter.use_scientific = False

        self.fig.yaxis.ticker = plot_tickers[1]
        self.fig.yaxis.major_label_text_font_size = "7pt"
        self.fig.yaxis.axis_label_text_font_size = '12pt'
        self.fig.yaxis.axis_label_text_font_style = 'normal'
        self.fig.yaxis.axis_label_standoff = 25
        self.fig.yaxis.minor_tick_line_width = 0
        self.fig.yaxis.major_tick_in = 0
        self.fig.yaxis.axis_line_color = '#444444'
        self.fig.yaxis.major_tick_line_color = '#444444'
        self.fig.yaxis.formatter.use_scientific = False

        self.fig.xgrid.visible = False
        self.fig.ygrid.visible = False

    # Example:
    #  plt = Plotter()
    #  plt.scatter([1, 2, 3, 4, 5], [6, 7, 2, 4, 5])
    #    or, plt.scatter([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], marker_size=[0.1, 0.2, 0.3, 0.4, 0.5])
    #  plt.show()
    def scatter(self, x, y, **kwargs):
        if 'marker_size' in kwargs:
            marker_size = kwargs['marker_size']
        else:
            marker_size = [(max(max(x), max(y)) - min(min(x), min(y))) / 100.0] * len(x)
        self.fig.circle(x, y, radius=marker_size, line_color="#BF4784", fill_color="#3AAB58", fill_alpha=0.8)

    def line(self, x, y, color, label, width, alpha):
        self.fig.line(x, y, color=color, legend_label=label, line_width=width, line_alpha=alpha)

    def bar(self):
        pass

    def show(self):
        show(self.fig)
