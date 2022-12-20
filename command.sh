cmake ../ -DROSS_BUILD_MODELS=ON; make; cp ../models/template-model/distr.txt ./models/template-model/; cd models/template-model/; ./model --robots=5; cd ../../
