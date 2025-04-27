import { ButtonHTMLAttributes } from "react";

type Props = ButtonHTMLAttributes<HTMLButtonElement> & {
  children: React.ReactNode;
};

export function MyButton({ children, ...props }: Props) {
  return (
    <button {...props} className="button font-bold text-black [--zoom:10]">
      {children}
    </button>
  );
}
