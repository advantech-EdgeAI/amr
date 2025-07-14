import argparse
import captioner_node

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-path", required=False, default="./script/config.yaml")
    args = parser.parse_args()

    captioner_node.main(args)