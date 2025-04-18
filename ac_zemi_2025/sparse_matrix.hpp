#pragma once

#include <vector>
#include <span>
#include <ranges>
#include <utility>
#include <algorithm>

#include "utility.hpp"

namespace ac_zemi_2025::sparse_matrix {
	struct Csr final {
		std::vector<usize> acc_row;
		std::vector<usize> col;

		auto row(const usize index) {
			auto f = [this](const usize i) -> usize {
				return this->col[i];
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto c_row(const usize index) const {
			auto f = [this](const usize i) -> usize {
				return this->col[i];
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto view() {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->row(i);
				});
		}

		auto c_view() const {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->c_row(i);
				});
		}

		auto begin() {
			return this->view().begin();
		}

		auto end() {
			return this->view().end();
		}

		auto cbegin() const {
			return this->c_view().begin();
		}

		auto cend() const {
			return this->c_view().end();
		}
	};

	struct Coo final {
		std::vector<std::pair<usize, usize>> buffer;
		bool is_sorted = false;

		static auto make(const usize buffer_size) {
			auto buffer = std::vector<std::pair<usize, usize>>{};
			buffer.reserve(buffer_size);
			return Coo {
				.buffer = std::move(buffer),
				.is_sorted = false
			};
		}

		void add(const std::pair<usize, usize>& index) {
			this->is_sorted = false;
			this->buffer.push_back(index);
		}

		void sort() {
			std::ranges::sort(this->buffer);
			this->is_sorted = true;
		}

		auto to_csr(const usize row_size) && -> Csr {
			if(not this->is_sorted) {
				this->sort();
			}

			std::vector<usize> acc_row{};
			std::vector<usize> col{};

			acc_row.reserve(row_size + 1);
			col.reserve(this->buffer.size());

			for(const auto& index : this->buffer) {
				col.push_back(index.second);
			}

			auto it = this->buffer.begin();
			for(usize i = 0; i < row_size; ++i) {
				acc_row.push_back(it - this->buffer.begin());
				while(it != this->buffer.end() && it->first == i) {
					++it;
				}
			}
			acc_row.push_back(this->buffer.size());

			return Csr {
				.acc_row = std::move(acc_row),
				.col = std::move(col)
			};
		}
	};
}
