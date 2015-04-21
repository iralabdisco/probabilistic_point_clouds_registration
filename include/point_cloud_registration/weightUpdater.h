class WeightUpdater :IterationCallback{
  WeightUpdater(double *rotation, double *translation, double dof, int dim);
  ~WeightUpdater() {}
  CallbackReturnType operator()(const IterationSummary& summary) = 0;
}