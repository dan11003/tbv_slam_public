# Statistics output for experiments
import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Read provided statistics file
bag_location = os.getenv('BAG_LOCATION')
directory = bag_location + '/CoralRadarEval/'
parser = argparse.ArgumentParser()
parser.add_argument("--filename", "-f", help="csv or text file containing statistics", default="")
args = parser.parse_args()

# Import statistics file
stats_file = directory + args.filename
data = pd.read_csv(stats_file)

# Directory to save figures
filename = args.filename
save_dir = directory + filename.replace(".txt","")
if not os.path.exists(save_dir) :
    os.mkdir(save_dir)

#  -- Plot 1 (accuracy by method) -- #

# Plot style (seaborn)
sns.set_theme(style="ticks", color_codes=True)
sns.set(style="ticks")
colors = ["#E59866", "#D35400", "#873600", "#85C1E9", "#3498DB", "#2874A6"]   #Daniel
color_figures = ["#E59866", "#D35400", "#873600", "#85C1E9", "#3498DB", "#2874A6","#FF90FA", "#FF00F3", "#8F0088", "#94FF76","#39FF00","#26AB00"]
marker_figures = ['o','o','o','o','o','o','o','o','o','o','o','o']
#colors = ["#E59866","#2874A6", "#D35400", "#85C1E9", "#3498DB",  "#873600"]  #Mine
data = data.rename(columns={'range error': 'Range error (m)'})
data = data.rename(columns={'scan spacing distance': 'Scan spacing distance (m)'})
data = data.rename(columns={'accuracy': 'Accuracy'})
data = data.rename(columns={'method': 'Method'})
data = data.rename(columns={'auc': 'AuC'})

pal = sns.color_palette(colors)
mar = ['o','o','o','s','s','s']
lin = ['-','--',':','-','--',':']

# Plot data
#data['method'] = data.apply(lambda row:  row['method'].replace('method', ''), axis=1)

#Lineplot (commented)
#sns_plot = sns.pointplot(x = 'Range error (m)', y = 'Accuracy', data = data, hue = 'Method', dodge = False, palette=pal)
#sns_plot.locator_params(axis='x', nbins=5)

# ----- Uncertainty plot (range error)
#sns_plot = sns.catplot(x='Range error (m)', y='Accuracy', hue='Method', kind="box", dodge=True, data=data, legend_out=True) # Old
method_scan = 'Method and scan spacing (m)'
data[method_scan] = data.apply(lambda row: row['Method'] + ', ' + str(row['Scan spacing distance (m)']), axis=1)
data_scan = data[ (data['Scan spacing distance (m)']==0) | (data['Scan spacing distance (m)']==5) | (data['Scan spacing distance (m)']==10) ]
data_scan_1 = data_scan[ data_scan['Method']=='P2P' ]  ; data_scan_1 = data_scan_1.sort_values('Scan spacing distance (m)')
data_scan_2 = data_scan[ data_scan['Method']=='P2D' ]  ; data_scan_2 = data_scan_2.sort_values('Scan spacing distance (m)')
data_scan_3 = data_scan[ data_scan['Method']=='P2L' ]  ; data_scan_3 = data_scan_3.sort_values('Scan spacing distance (m)')
data_scan_4 = data_scan[ data_scan['Method']=='Coral'] ; data_scan_4 = data_scan_4.sort_values('Scan spacing distance (m)')

sns_plot = sns.lineplot(x="Range error (m)", y="Accuracy", hue=method_scan, style=method_scan, data=data_scan_1, markers=['.','o','s'], dashes=False, palette=["#E59866", "#D35400", "#873600"]) # Current
sns_plot = sns.lineplot(x="Range error (m)", y="Accuracy", hue=method_scan, style=method_scan, data=data_scan_2, markers=['.','o','s'], dashes=False, palette=["#85C1E9", "#3498DB", "#2874A6"]) 
sns_plot = sns.lineplot(x="Range error (m)", y="Accuracy", hue=method_scan, style=method_scan, data=data_scan_3, markers=['.','o','s'], dashes=False, palette=["#FF90FA", "#FF00F3", "#8F0088"]) 
sns_plot = sns.lineplot(x="Range error (m)", y="Accuracy", hue=method_scan, style=method_scan, data=data_scan_4, markers=['.','o','s'], dashes=False, palette=["#94FF76","#39FF00","#26AB00"]) 
#plt.ylim(0.45,1)
#plt.xlim(0.05,0.95)

plt.grid()
plt.show()
#fig = sns_plot.get_figure()
#fig.savefig(save_dir + '/accuracyByMethod.pdf', format='pdf')

# ----- Uncertainty plot (scan spacing)
#sns_plot = sns.catplot(x='Range error (m)', y='Accuracy', hue='Method', kind="box", dodge=False, data=data, legend_out=True) # Old
method_range = 'Method and range error (m)'
data[method_range] = data.apply(lambda row: row['Method'] + ', ' + str(row['Range error (m)']), axis=1)
data_spacing = data[ (data['Range error (m)']==0.3) | (data['Range error (m)']==0.6) | (data['Range error (m)']==0.9)]
data_spacing_1 = data_spacing[data_spacing['Method']=='P2P']
data_spacing_2 = data_spacing[data_spacing['Method']=='P2D']
data_spacing_3 = data_spacing[data_spacing['Method']=='P2L']
data_spacing_4 = data_spacing[data_spacing['Method']=='Coral']
sns_plot = sns.lineplot(x="Scan spacing distance (m)", y="Accuracy", hue=method_range, style=method_range, data=data_spacing_1, markers=['.','o','s'], dashes=False, palette=["#E59866", "#D35400", "#873600"]) # Current
sns_plot = sns.lineplot(x="Scan spacing distance (m)", y="Accuracy", hue=method_range, style=method_range, data=data_spacing_2, markers=['.','o','s'], dashes=False, palette=["#85C1E9", "#3498DB", "#2874A6"])
sns_plot = sns.lineplot(x="Scan spacing distance (m)", y="Accuracy", hue=method_range, style=method_range, data=data_spacing_3, markers=['.','o','s'], dashes=False, palette=["#FF90FA", "#FF00F3", "#8F0088"])
sns_plot = sns.lineplot(x="Scan spacing distance (m)", y="Accuracy", hue=method_range, style=method_range, data=data_spacing_4, markers=['.','o','s'], dashes=False, palette=["#94FF76","#39FF00","#26AB00"])
#sns_plot = sns.lineplot(x="scan spacing distance", y="Accuracy", hue="Method", style="Method", data= data[data["Range error (m)"]==0.5], markers=['s','s','s'], dashes=False, palette=["#85C1E9", "#3498DB", "#2874A6"]) # Current
#sns_plot = sns.lineplot(x="scan spacing distance", y="Accuracy", hue="Method", style="Method", data= data[data["Range error (m)"]==0.9], markers=['^','^','^'], dashes=False, palette=["#E59866", "#D35400", "#873600"]) # Current
#plt.ylim(0.45,1)
#plt.xlim(0.05,0.95)

plt.grid()
plt.show()
#fig = sns_plot.get_figure()
#fig.savefig(save_dir + '/accuracyByMethod.pdf', format='pdf')


# Print parameters for 'best' configuration for each method
#data_p2p = data[data['Method'] == 'P2P']
#data_p2d = data[data['Method'] == 'P2D']
#data_p2l = data[data['Method'] == 'P2L']

#print('Best configurations for all methods: ') 
#print(data_p2p[data_p2p.Accuracy == data_p2p.Accuracy.max()]) ; print(' ')
#print(data_p2d[data_p2d.Accuracy == data_p2d.Accuracy.max()]) ; print(' ')
#print(data_p2l[data_p2l.Accuracy == data_p2l.Accuracy.max()]) ; print(' ')

# Box plot (accuracies and AuC for all methods)
#sns_plot_2 = sns.boxplot(x=data['Method'], y=data['Accuracy'], width=0.3)
#plt.show()
#fig = sns_plot_2.get_figure()
#fig.savefig(save_dir + '/accuracyBoxplot.pdf', format='pdf')

#sns_plot_3 = sns.boxplot(x=data['Method'], y=data['AuC'], width=0.3)
#plt.show()
#fig = sns_plot_3.get_figure()
#fig.savefig(save_dir + '/aucBoxplot.pdf', format='pdf')




#  --- OLD CODE (standalone matplot lib, no seaborn) --- #

# Filter data by method
# data_p2p = data[data['method'] == 'P2P']
# data_p2d = data[data['method'] == 'P2D']
# data_p2l = data[data['method'] == 'P2L']

# # Get data
# x_p2p = data_p2p['range error'] ; y_p2p = data_p2p['accuracy']
# x_p2d = data_p2d['range error'] ; y_p2d = data_p2d['accuracy']
# x_p2l = data_p2l['range error'] ; y_p2l = data_p2l['accuracy']

# # Plot data
# plt.plot(x_p2p,y_p2p,marker='o',label='P2P')
# plt.plot(x_p2d,y_p2d,marker='o',label='P2D')
# plt.plot(x_p2l,y_p2l,marker='o',label='P2L')
# plt.legend(loc="upper left")
# plt.xlabel('Range error (m)')
# plt.ylabel('Classification accuracy')
# plt.savefig(save_dir + '/accuracyByMethod.pdf')
# plt.show()

# # -- Plot 2 (boxplot by method) -- #

# accuracies = [y_p2p, y_p2d, y_p2l]
# plt.boxplot(accuracies)
# plt.xticks([1, 2, 3], ['P2P','P2D','P2L'])
# plt.ylabel('Classification accuracy')
# plt.savefig(save_dir + '/boxplotByMethod.pdf')
# plt.show()