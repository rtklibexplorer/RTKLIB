Instructions for generating 2023 GSDC solution submission file (ensemble from two solution sets)

I) Generate rinex and RTKLIB solution files for both solutions
	1) Configure file header in run_ppk_multi.py script
		a) set OVERWRITE_RINEX = TRUE
		b) set soltag_rtklib to desired solution name

	2) Run run_ppk_multi.py script twice, once for each base station pair
		a) run1: bases = ['slac', 'vdcy'] (Set this in script header)
		b) run2: bases = ['p222', 'torp'] (Set this in script header)
	
	3) Run filter_ride.py to filter problematic solutions
		a) set datafile (in script header )to match the solution name from step 1

II) Generate solution csv files for both solutions
	1) Run create_baseline_CSV_from_pos.py twice
		a) run1: bases = ['slac', 'vdcy', 'p222', 'torp']  (Set this in script header)
		b) rename the output file (xxx.csv) to xxxa.csv 
		c) run2: bases = ['p222', 'torp', 'p222', 'torp']  (Set this in script header)
		d) rename the output file (xxx.csv) to xxxb.csv 
	
III) Merge both solutions into a single csv file
	1) Run merge_baselines.py
		a) set mergefiles (in script header) to the two csv files generated in step 4

IV)	Create a submission csv file from the merged solution csv file	
	6) Run create_submission.py
